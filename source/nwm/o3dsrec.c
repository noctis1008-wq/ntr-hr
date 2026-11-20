/*
 * o3ds-rec: 旧3DSで画面を SD カードへ録画する（https://github.com/noctis1008-wq/o3ds-rec）
 *
 * 旧3DSの CPU では JPEG 圧縮が間に合わない（3〜7fps）ため、圧縮せずに
 * 「前回から変わったライン（画面の縦1列）だけ」を .3dr 形式で SD に書く。
 * PC 側の tools/convert/3dr2mp4.py で MP4 に変換する。
 *
 * 構成:
 *   キャプチャ側（nwm のエンコードスレッド）: o3dsrecFrame()
 *     ライン毎ハッシュで差分を取り、リングバッファに詰める（CPU は memcmp 程度）
 *   書き込みスレッド: writerThread()
 *     リングバッファから 256KB 単位で SD に書く。実機計測（docs/bench/）で
 *     256KB + ファイル事前確保が最も待ち時間が短かったため。
 *
 * ファイル: sdmc:/o3ds-rec/REC_YYYYMMDD_HHMMSS.3dr（3.5GB ごとに .partN.3dr に分割）
 */

#include "global.h"
#include "3ds/services/fs.h"

#include "o3dsrec.h"
#include "rec_core.h"

#include <stdarg.h>
#include <string.h>

#define REC_DIR "/o3ds-rec"
#define BLOCK_SIZE (256 * 1024)
/* 1フレームの最大サイズ（上画面 RGB565 で約 188KB。RGB565 変換を常に有効にするので 2bpp 上限） */
#define FRAME_MAX (200 * 1024)
#define PREALLOC_STEP (64ull * 1024 * 1024)
#define SPLIT_LIMIT (3584ull * 1024 * 1024)
#define KEY_INTERVAL 300
#define WRITER_STACK_SIZE 0x2000
#define WRITER_PRIO 0x20

#define TOP_LINES 400
#define BOT_LINES 320

/* ---- 一度だけ確保するもの ---- */
static u8 *ring;          /* ringSize + FRAME_MAX（末尾ははみ出し用） */
static u32 ringSize;      /* BLOCK_SIZE の 2 のべき乗倍 */
static u32 *writerStack;
static FS_Archive sdmc;
static bool sdmcOpened;

/* ---- 録画1回ぶんの状態 ---- */
static bool active;
static u32 lastFormat;
static u32 logFrames; /* 次に状況をログに出すフレーム数 */
static bool startFailed; /* 開始に失敗したら o3dsrecStop() まで再試行しない */
static rec_screen_state stTop, stBot;
static u64 hashTop[TOP_LINES], hashBot[BOT_LINES];
static u32 prodPos;  /* 書き込んだ総バイト（キャプチャ側が更新、u32 で回ってよい） */
static u32 writePos; /* SD に書き終えた総バイト（書き込みスレッドが更新） */
static u32 drops;
static u64 startTick;

static Handle writerThread;
static bool inlineWrite; /* 書き込みスレッドを作れなかったときはキャプチャ側で書く */
static Handle wakeEvent;
static volatile u32 stopReq;
static volatile u32 writerFailed;

/* 書き込みスレッドだけが触る */
static Handle file;
static u64 fileOff;
static u64 fileAlloc;
static u32 partNo;
static char baseName[64];

static u32 loadAcq(u32 *p) { return __atomic_load_n(p, __ATOMIC_ACQUIRE); }
static void storeRel(u32 *p, u32 v) { __atomic_store_n(p, v, __ATOMIC_RELEASE); }

/* 診断ログ sdmc:/o3ds-rec/log.txt（どこで止まったかを実機で調べるため） */
static void recLog(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
static void recLog(const char *fmt, ...)
{
	if (!sdmcOpened)
		return;
	char buf[160];
	va_list ap;
	va_start(ap, fmt);
	u32 t = (u32)(svcGetSystemTick() / (SYSCLOCK_ARM11 / 1000));
	int n = xsnprintf(buf, sizeof(buf), "[%lu] ", (unsigned long)t);
	n += xvsnprintf(buf + n, sizeof(buf) - n, fmt, ap);
	va_end(ap);
	if (n > (int)sizeof(buf) - 2)
		n = sizeof(buf) - 2;
	buf[n++] = '\n';

	Handle f;
	if (FSUSER_OpenFile(&f, sdmc, fsMakePath(PATH_ASCII, REC_DIR "/log.txt"), FS_OPEN_WRITE | FS_OPEN_CREATE, 0) != 0)
		return;
	u64 size = 0;
	FSFILE_GetSize(f, &size);
	u32 w;
	FSFILE_Write(f, &w, size, buf, n, FS_WRITE_FLUSH);
	FSFILE_Close(f);
}

/* ---- 日付（osGetTime は 1900-01-01 からのミリ秒） ---- */
static void civilFromDays(s32 z, s32 *y, u32 *m, u32 *d)
{
	/* Howard Hinnant の days_from_civil の逆変換（z は 1970-01-01 からの日数） */
	z += 719468;
	s32 era = (z >= 0 ? z : z - 146096) / 146097;
	u32 doe = (u32)(z - era * 146097);
	u32 yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
	s32 yy = (s32)yoe + era * 400;
	u32 doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
	u32 mp = (5 * doy + 2) / 153;
	*d = doy - (153 * mp + 2) / 5 + 1;
	*m = mp < 10 ? mp + 3 : mp - 9;
	*y = yy + (*m <= 2);
}

static void makeBaseName(void)
{
	u64 sec = osGetTime() / 1000;
	const u64 unixFrom1900 = 2208988800ull;
	sec = sec > unixFrom1900 ? sec - unixFrom1900 : 0;
	s32 y;
	u32 m, d;
	civilFromDays((s32)(sec / 86400), &y, &m, &d);
	u32 t = (u32)(sec % 86400);
	xsnprintf(baseName, sizeof(baseName), REC_DIR "/REC_%04d%02lu%02lu_%02lu%02lu%02lu",
		(int)y, (unsigned long)m, (unsigned long)d,
		(unsigned long)(t / 3600), (unsigned long)(t / 60 % 60), (unsigned long)(t % 60));
}

/* ---- ファイル操作（書き込みスレッド・開始時のみ） ---- */
static bool openPart(void)
{
	char path[96];
	if (partNo <= 1)
		xsnprintf(path, sizeof(path), "%s.3dr", baseName);
	else
		xsnprintf(path, sizeof(path), "%s.part%lu.3dr", baseName, (unsigned long)partNo);

	Result res = FSUSER_OpenFile(&file, sdmc, fsMakePath(PATH_ASCII, path),
		FS_OPEN_WRITE | FS_OPEN_CREATE, 0);
	if (res != 0) {
		nsDbgPrint("o3ds-rec: open %s failed: %08"PRIx32"\n", path, res);
		file = 0;
		return false;
	}

	u8 hdr[REC_FILE_HEADER_SIZE] ALIGNED(4);
	rec_write_file_header(hdr, startTick, 0);
	u32 written = 0;
	res = FSFILE_Write(file, &written, 0, hdr, sizeof(hdr), 0);
	if (res != 0 || written != sizeof(hdr)) {
		nsDbgPrint("o3ds-rec: header write failed: %08"PRIx32"\n", res);
		FSFILE_Close(file);
		file = 0;
		return false;
	}
	fileOff = sizeof(hdr);
	fileAlloc = PREALLOC_STEP;
	FSFILE_SetSize(file, fileAlloc); /* 失敗しても書き込み自体はできる */
	return true;
}

static void closePart(void)
{
	if (!file)
		return;
	FSFILE_SetSize(file, fileOff); /* 事前確保した残りを切り捨てる */
	FSFILE_Flush(file);
	FSFILE_Close(file);
	file = 0;
}

static bool writeBytes(const u8 *buf, u32 len)
{
	if (len == 0)
		return true;
	if (fileOff + len > SPLIT_LIMIT) {
		closePart();
		++partNo;
		if (!openPart())
			return false;
	}
	if (fileOff + len > fileAlloc) {
		fileAlloc += PREALLOC_STEP;
		FSFILE_SetSize(file, fileAlloc);
	}
	u32 written = 0;
	Result res = FSFILE_Write(file, &written, fileOff, buf, len, 0);
	if (res != 0 || written != len) {
		nsDbgPrint("o3ds-rec: write failed at %lu KB: %08"PRIx32"\n", (unsigned long)(fileOff >> 10), res);
		return false;
	}
	fileOff += len;
	return true;
}

/* 溜まった完全なブロックを書く。false = 書き込み失敗 */
static bool flushBlocks(void)
{
	u32 wp = writePos;
	for (;;) {
		u32 pp = loadAcq(&prodPos);
		if (pp - wp < BLOCK_SIZE)
			return true;
		if (!writeBytes(ring + (wp & (ringSize - 1)), BLOCK_SIZE))
			return false;
		wp += BLOCK_SIZE;
		storeRel(&writePos, wp);
	}
}

/* 最後の端数を書いてファイルを閉じる */
static bool flushTail(void)
{
	/* 残り（ブロック未満）。ブロック境界から始まるので連続している */
	u32 wp = writePos, pp = loadAcq(&prodPos);
	bool ok = writeBytes(ring + (wp & (ringSize - 1)), pp - wp);
	if (ok)
		storeRel(&writePos, pp);
	closePart();
	return ok;
}

static void writerMain(void *)
{
	for (;;) {
		svcWaitSynchronization(wakeEvent, 50 * 1000 * 1000);

		if (!flushBlocks()) {
			writerFailed = 1;
			closePart();
			break;
		}
		if (stopReq) {
			if (!flushTail())
				writerFailed = 1;
			break;
		}
	}
	svcExitThread();
}

/* ---- 開始・終了 ---- */
static bool ensureMemory(void)
{
	if (ring)
		return true;
	/* 旧3DSの nwm は空きメモリが少ないので、取れる大きさで妥協する */
	static const u32 sizes[] = { 512 * 1024, 256 * 1024 };
	for (unsigned i = 0; i < sizeof(sizes) / sizeof(*sizes); ++i) {
		u8 *m = (u8 *)plgRequestMemory(sizes[i] + FRAME_MAX);
		if (m) {
			ring = m;
			ringSize = sizes[i];
			break;
		}
	}
	if (!ring) {
		nsDbgPrint("o3ds-rec: out of memory\n");
		recLog("ERROR out of memory (ring)");
		return false;
	}
	writerStack = (u32 *)plgRequestMemory(WRITER_STACK_SIZE);
	if (!writerStack) {
		nsDbgPrint("o3ds-rec: out of memory (stack)\n");
		recLog("ERROR out of memory (stack)");
		return false;
	}
	nsDbgPrint("o3ds-rec: ring buffer %lu KB\n", (unsigned long)(ringSize / 1024));
	recLog("memory ok: ring %lu KB, total used %lu KB", (unsigned long)(ringSize / 1024),
		(unsigned long)(plgGetMemoryUsage() / 1024));
	return true;
}

static bool ensureSd(void)
{
	if (sdmcOpened)
		return true;
	if (*fsGetSessionHandle() == 0) {
		Result res = fsInit();
		if (res != 0) {
			nsDbgPrint("o3ds-rec: fsInit failed: %08"PRIx32"\n", res);
			return false;
		}
	}
	Result res = FSUSER_OpenArchive(&sdmc, ARCHIVE_SDMC, fsMakePath(PATH_EMPTY, ""));
	if (res != 0) {
		nsDbgPrint("o3ds-rec: open SD failed: %08"PRIx32"\n", res);
		return false;
	}
	sdmcOpened = true;
	return true;
}

static bool start(void)
{
	if (!ensureSd())
		return false;
	FSUSER_CreateDirectory(sdmc, fsMakePath(PATH_ASCII, REC_DIR), 0); /* 既にあれば失敗するが問題ない */
	recLog("start: SD ok (format %lu)", (unsigned long)lastFormat);
	if (!ensureMemory())
		return false;

	startTick = svcGetSystemTick();
	makeBaseName();
	partNo = 1;
	if (!openPart()) {
		recLog("ERROR cannot open %s.3dr", baseName);
		return false;
	}
	recLog("file %s.3dr opened", baseName);

	rec_screen_init(&stTop, REC_SCREEN_TOP_LEFT, TOP_LINES, KEY_INTERVAL);
	rec_screen_init(&stBot, REC_SCREEN_BOTTOM, BOT_LINES, KEY_INTERVAL);
	stTop.to_rgb565 = stBot.to_rgb565 = 1; /* 24bit 色も RGB565 で保存（30fps 優先） */
	prodPos = writePos = 0;
	drops = 0;
	stopReq = 0;
	writerFailed = 0;

	inlineWrite = false;
	Result res = svcCreateEvent(&wakeEvent, RESET_ONESHOT);
	if (res == 0) {
		res = svcCreateThread(&writerThread, writerMain, 0,
			&writerStack[WRITER_STACK_SIZE / 4 - 10], WRITER_PRIO, -2);
		if (res != 0) {
			svcCloseHandle(wakeEvent);
			wakeEvent = 0;
		}
	}
	if (res != 0) {
		/* nwm のスレッド数上限など。遅くなるがキャプチャ側で直接書く */
		nsDbgPrint("o3ds-rec: no writer thread (%08"PRIx32"), writing inline\n", res);
		writerThread = 0;
		inlineWrite = true;
	}
	recLog(inlineWrite ? "writer: inline (%08lx)" : "writer: thread ok (%08lx)", (unsigned long)res);
	logFrames = 1;

	active = true;
	nsDbgPrint("o3ds-rec: recording to %s.3dr\n", baseName);
	return true;
}

void o3dsrecStop(void)
{
	if (active) {
		/* 終端チャンク（入らなければ省略。変換ツールはなくても読める） */
		u32 pp = prodPos;
		if (pp + REC_CHUNK_HEADER_SIZE - loadAcq(&writePos) <= ringSize) {
			u32 off = pp & (ringSize - 1);
			u32 n = rec_write_eos(ring + off, svcGetSystemTick());
			if (off + n > ringSize)
				memcpy(ring, ring + ringSize, off + n - ringSize);
			storeRel(&prodPos, pp + n);
		}

		if (inlineWrite) {
			if (!flushBlocks() || !flushTail())
				writerFailed = 1;
			closePart();
		} else {
			stopReq = 1;
			svcSignalEvent(wakeEvent);
			svcWaitSynchronization(writerThread, 10ll * 1000 * 1000 * 1000);
			svcCloseHandle(writerThread);
			svcCloseHandle(wakeEvent);
			writerThread = 0;
			wakeEvent = 0;
		}
		active = false;
		nsDbgPrint("o3ds-rec: stopped (%lu frames, %lu dropped)\n",
			(unsigned long)(stTop.frame_count + stBot.frame_count), (unsigned long)drops);
		recLog("stopped: %lu frames, %lu dropped, %lu KB, writer %s",
			(unsigned long)(stTop.frame_count + stBot.frame_count), (unsigned long)drops,
			(unsigned long)(writePos / 1024), writerFailed ? "FAILED" : "ok");
	}
	startFailed = false;
}

void o3dsrecFrame(const u8 *src, u32 format, u32 isTop)
{
	if (!active) {
		if (startFailed)
			return;
		lastFormat = format;
		if (!start()) {
			startFailed = true;
			return;
		}
	}

	if (writerFailed) {
		/* SD が一杯・抜かれた等。録画を終えて、設定が変わるまで再開しない */
		o3dsrecStop();
		startFailed = true;
		return;
	}

	format &= 7;
	if (format > REC_FMT_RGBA4)
		return;

	u32 pp = prodPos;
	if (pp + FRAME_MAX - loadAcq(&writePos) > ringSize) {
		/* SD が追いついていない: このフレームは捨てる（ハッシュは進めないので次の差分は正しい） */
		++drops;
		if (!inlineWrite)
			svcSignalEvent(wakeEvent);
		return;
	}

	u32 off = pp & (ringSize - 1);
	u32 pitch = REC_LINE_PX * rec_bpp((u8)format);
	size_t n = isTop
		? rec_encode_frame_hashed(&stTop, src, pitch, (u8)format, svcGetSystemTick(), ring + off, hashTop)
		: rec_encode_frame_hashed(&stBot, src, pitch, (u8)format, svcGetSystemTick(), ring + off, hashBot);
	if (n == 0)
		return; /* 前回と同じ画面 */

	if (off + n > ringSize)
		memcpy(ring, ring + ringSize, off + n - ringSize);

	storeRel(&prodPos, pp + (u32)n);
	u32 frames = stTop.frame_count + stBot.frame_count;
	if (frames >= logFrames) {
		recLog("frames %lu, dropped %lu, produced %lu KB, written %lu KB", (unsigned long)frames,
			(unsigned long)drops, (unsigned long)(pp / 1024), (unsigned long)(loadAcq(&writePos) / 1024));
		logFrames = logFrames < 300 ? logFrames * 10 : logFrames + 900;
	}
	if (inlineWrite) {
		if (!flushBlocks())
			writerFailed = 1;
	} else if ((pp ^ (pp + (u32)n)) & ~(u32)(BLOCK_SIZE - 1)) {
		svcSignalEvent(wakeEvent); /* ブロック境界をまたいだ */
	}
}
