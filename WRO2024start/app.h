/*
 *  TOPPERS/ASP Kernel
 *      Toyohashi Open Platform for Embedded Real-Time Systems/
 *      Advanced Standard Profile Kernel
 *
 *  Copyright (C) 2000-2003 by Embedded and Real-Time Systems Laboratory
 *                              Toyohashi Univ. of Technology, JAPAN
 *  Copyright (C) 2004-2010 by Embedded and Real-Time Systems Laboratory
 *              Graduate School of Information Science, Nagoya Univ., JAPAN
 *
 *  上記著作権者は，以下の(1)〜(4)の条件を満たす場合に限り，本ソフトウェ
 *  ア（本ソフトウェアを改変したものを含む．以下同じ）を使用・複製・改
 *  変・再配布（以下，利用と呼ぶ）することを無償で許諾する．
 *  (1) 本ソフトウェアをソースコードの形で利用する場合には，上記の著作
 *      権表示，この利用条件および下記の無保証規定が，そのままの形でソー
 *      スコード中に含まれていること．
 *  (2) 本ソフトウェアを，ライブラリ形式など，他のソフトウェア開発に使
 *      用できる形で再配布する場合には，再配布に伴うドキュメント（利用
 *      者マニュアルなど）に，上記の著作権表示，この利用条件および下記
 *      の無保証規定を掲載すること．
 *  (3) 本ソフトウェアを，機器に組み込むなど，他のソフトウェア開発に使
 *      用できない形で再配布する場合には，次のいずれかの条件を満たすこ
 *      と．
 *    (a) 再配布に伴うドキュメント（利用者マニュアルなど）に，上記の著
 *        作権表示，この利用条件および下記の無保証規定を掲載すること．
 *    (b) 再配布の形態を，別に定める方法によって，TOPPERSプロジェクトに
 *        報告すること．
 *  (4) 本ソフトウェアの利用により直接的または間接的に生じるいかなる損
 *      害からも，上記著作権者およびTOPPERSプロジェクトを免責すること．
 *      また，本ソフトウェアのユーザまたはエンドユーザからのいかなる理
 *      由に基づく請求からも，上記著作権者およびTOPPERSプロジェクトを
 *      免責すること．
 * 
 *  本ソフトウェアは，無保証で提供されているものである．上記著作権者お
 *  よびTOPPERSプロジェクトは，本ソフトウェアに関して，特定の使用目的
 *  に対する適合性も含めて，いかなる保証も行わない．また，本ソフトウェ
 *  アの利用により直接的または間接的に生じたいかなる損害に関しても，そ
 *  の責任を負わない．
 *
 *  $Id: sample1.h 2416 2012-09-07 08:06:20Z ertl-hiro $
 */

/*
 *		サンプルプログラム(1)のヘッダファイル
 */

/*
 *  ターゲット依存の定義
 */
#include "target_test.h"

/*
 *  各タスクの優先度の定義
 */

#define MAIN_PRIORITY	5		/* メインタスクの優先度 */
								/* HIGH_PRIORITYより高くすること */

#define HIGH_PRIORITY	9		/* 並行実行されるタスクの優先度 */
#define MID_PRIORITY	10
#define LOW_PRIORITY	11

#define PAI             3.1415
#define RADIUS          6.24
#define ROBOT1CM        18.364

#define MSEC            1000

typedef enum port {
    RIGHT,
    LEFT,
    BOTH
} port_t ;

typedef enum armmode {
    RIGHTDOWN,
    LEFTDOWN,
    RIGHTDOWNRIGHT,
    LEFTDOWNLEFT
} armmode_t ;

typedef enum armmode_new {
    UP,
    DOWN,
    SETNEW,
    SETNEW_2,
    SETNEWSHIP,
    CLOSESHIP,
    OBJDOWN,
    OPEN,
    CLOSE,
    SETSHIP,
    SETOPEN,
    SETCLOSE,
    SET,
    GET_OBJ_2
} armmode_new_t ;


/*
 *  ターゲットに依存する可能性のある定数の定義
 */

#ifndef STACK_SIZE
#define	STACK_SIZE		4096		/* タスクのスタックサイズ */
#endif /* STACK_SIZE */

#ifndef LOOP_REF
#define LOOP_REF		ULONG_C(1000000)	/* 速度計測用のループ回数 */
#endif /* LOOP_REF */

/*
 *  関数のプロトタイプ宣言
 */
#ifndef TOPPERS_MACRO_ONLY

extern void	task(intptr_t exinf);
extern void	main_task(intptr_t exinf);
extern void balance_task(intptr_t exinf);
extern void idle_task(intptr_t exinf);
//extern void	tex_routine(TEXPTN texptn, intptr_t exinf);
//#ifdef CPUEXC1
//extern void	cpuexc_handler(void *p_excinf);
//#endif /* CPUEXC1 */
//extern void	cyclic_handler(intptr_t exinf);
//extern void	alarm_handler(intptr_t exinf);
//
//extern void	gpio_handler_initialize(intptr_t exinf);
//extern void	gpio_handler(void);
extern void	gpio_irq_dispatcher(intptr_t exinf);
//
//extern void	uart_sensor_monitor(intptr_t exinf);
//
//extern void	ev3_uart_cyclic_handler(intptr_t exinf);
//extern void	ev3_uart_daemon(intptr_t exinf);
//extern void	ev3_uart_port2_irq(void);
//
//extern void initialize_ev3(intptr_t exinf);
#endif /* TOPPERS_MACRO_ONLY */
