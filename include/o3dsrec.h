#ifndef O3DSREC_H
#define O3DSREC_H

/*
 * o3ds-rec: 旧3DSでリモートプレイの代わりに画面を SD カードへ録画する。
 * https://github.com/noctis1008-wq/o3ds-rec
 *
 * nwm（旧3DS版）のエンコードスレッドから呼ぶ。
 */

#include "3ds/types.h"

/* キャプチャ済みの画面を1枚渡す（src はライン詰め、1ライン = 240px）。
 * 録画していなければ自動で開始する。 */
void o3dsrecFrame(const u8 *src, u32 format, u32 isTop);

/* 録画中ならファイルを閉じて終える（何度呼んでもよい） */
void o3dsrecStop(void);

#endif
