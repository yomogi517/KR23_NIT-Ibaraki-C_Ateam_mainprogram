/* ------------------------------------------------------------------------------------------------- */
/* Kouryuu ROBOCON 2023 Ibaraki Kosen A team main program                                            */
/* MainBoard: Nucleo F446RE (開発環境:MbedStudio)                                                    */
/* MotorDriver: 通称黒MD (自作モータードライバ 2017年に設計・製造、駆動方式:LAP、通信方式:I2C)         */
/* Actuator: RS-555 1/71 * 5     パワーウィンドウモーターJC/LC-578VA * 1                              */
/* Sensor: Limitswitch * 5                                                                           */
/* ------------------------------------------------------------------------------------------------- */

#include "mbed.h"
#include "QEI.h"
#include "PS3.h"
#include <cstdint>  //uint8 (char) を使用したら勝手に追加された

// マクロでモータードライバのアドレス管理（I2Cのアドレス用）
#define M_R_WHEEL   0x40
#define M_L_WHEEL   0x42
#define M_L_TRAY    0x44    // 下のトレー
#define M_R_TRAY    0x46    // 上のトレー
#define M_VERT_ARM  0x50
#define M_HORI_ARM  0x52

// mbedのライブラリThread ドキュメントはこちらから:https://os.mbed.com/docs/mbed-os/v6.16/apis/thread.html
Thread th1;
Thread th2;
int thread1();  // デバック用printfを100msごとにまわすためのThread
int thread2();  // センサーをポーリングで読むためのThread

// 使用するピンおよび機能の宣言
I2C i2c(D14,D15); // D14 = PB_8, D15 = PB_9
PS3 ps3( A0, A1); // PA_0, PA_1

DigitalIn button(BUTTON1);  // Nucleo上にある青いボタン(デバック用)
// リミットスイッチ用DigitalIn(基板上でプルアップしているため、リミットスイッチの配線をNC,COMにつなぐと通常時０,押されたとき１、NO,COMにつなぐと通常時１,押されたとき０)
// 今回の機体はNC,COMに配線
DigitalIn sw1(PC_4);        // 昇降SW上
DigitalIn sw2(PB_15);       // 昇降SW下
DigitalIn sw3(PB_12);       // アーム上SW
DigitalIn sw4(PC_5);        // アーム左右SW1
DigitalIn sw5(PC_8);        // アーム左右sw2


// InterruptIn sw1(PC_4);        // 昇降SW下（左）
// InterruptIn sw2(PB_15);       // 昇降SW上（右）
// InterruptIn sw3(PB_12);       // アーム上SW
// InterruptIn sw4(PC_5);        // アーム左右SW1機体内側(hori_in)
// InterruptIn sw5(PC_8);        // アーム左右sw2機体外側(hori_out)

DigitalOut EMG(D13);    // 遠隔緊急停止
DigitalOut myled(LED1); // Nucleo上にあるチップLED
DigitalOut led1(PA_9);  //  PA_9 = D8
DigitalOut led2(PA_8);  //  PA_8 = D7
DigitalOut led3(PB_10); // PB_10 = D6
DigitalOut led4(PB_4);  //  PB_4 = D5
DigitalOut led5(PB_5);  //  PB_5 = D4

/* ロータリーエンコーダ_外部ライブラリQEIを使用(機体にエンコーダ自体を積まなかったためコメントアウト)
QEI rori1(  PD_2, PC_11, NC, 2048, QEI::X2_ENCODING);
QEI rori2( PC_10, PC_12, NC, 2048, QEI::X2_ENCODING);
QEI rori3( PA_13, PA_14, NC, 2048, QEI::X2_ENCODING);
QEI rori4( PA_15,  PB_7, NC, 2048, QEI::X2_ENCODING);
QEI rori5(  PH_0,  PH_1, NC, 2048, QEI::X2_ENCODING);
QEI rori6(  PC_3,  PA_4, NC, 2048, QEI::X2_ENCODING);
*/

void send(char, char);                      // MDにI2Cでデータを送る関数
int outputMotorData_R_W (char *data);
int outputMotorData_L_W (char *data);
int outputMotorData_Arm_hori (char *data);

/* InterruptInを使用時の割り込み処理関数
void sw_tray_sita_func ();
void sw_tray_ue_func ();
void sw_arm_ue_func ();
void sw_arm_hori_in_func ();
void sw_arm_hori_out_func ();
*/

/* エンコーダ用変数
int pulse1, pulse2, pulse3, pulse4, pulse5, pulse6;
float angle1, angle2, angle3, angle4, angle5, angle6;
*/

// 下記データを管理する変数は種類や機構ごとに構造体で管理したほうが変数の管理がしやすいと思います（大会前日の調整時にきづいた）
char R_W_Mdata, L_W_Mdata; // 足回りのMDに送るPWMデータ
char Arm_hori_Mdata; // アーム左右MDに送るPWMデータ
int sw_tray_ue, sw_tray_sita, sw_arm_ue, sw_arm_hori_in, sw_arm_hori_out, user_button; // printf用変数

struct sw_flag { // 構造体でトレー昇降機構の状態管理変数を作成、０のときbreak、１のとき上昇、-1のとき下降
    int flag;
};

int main (void){
    
    struct sw_flag f_Left, f_Right;

    f_Left.flag = 0;
    f_Right.flag = 0;

    EMG = 0;

    // ショートブレーキ(break)状態へ
    R_W_Mdata = 0x80;
    L_W_Mdata = 0x80;
    Arm_hori_Mdata = 0x80;

    // Thread開始
    th1.start(thread1);
    th2.start(thread2);

    /* エンコーダリセット
    rori1.reset();
    rori2.reset();
    rori3.reset();
    rori4.reset();
    rori5.reset();
    rori6.reset();
    */

    while (true){

        if(ps3.getSELECTState()){
            EMG = 1;
        }
        if(ps3.getSTARTState()){
            EMG = 0;
        }

        outputMotorData_L_W(&L_W_Mdata);
        outputMotorData_R_W(&R_W_Mdata);
        send(M_R_WHEEL, 256 - L_W_Mdata);
        send(M_L_WHEEL, R_W_Mdata);

        outputMotorData_Arm_hori(&Arm_hori_Mdata);
        send(M_HORI_ARM, Arm_hori_Mdata);

        if( ps3.getButtonState(PS3::ue) && !sw_arm_ue ){
            // send(M_VERT_ARM, 0x58);      // 128-40の出力（適当だがあまり電圧必要ないと判断）
            // send(M_VERT_ARM, 0x62);      // 128-30
            send(M_VERT_ARM, 0x71);         // 128-15
            // send(M_VERT_ARM, 0x44);      // 128-60
        } else if ( ps3.getButtonState(PS3::sita) ){
            // send(M_VERT_ARM, 0xa8);      // 128+40の出力（同上）
            send(M_VERT_ARM, 0x9e);         // 128+30
            // send(M_VERT_ARM, 0xbc);      // 128+60
        } else {
            send(M_VERT_ARM, 0x80);     // break
        }

        if ( ps3.getButtonState(PS3::sankaku) ){
            f_Right.flag = 1;
        } else if ( ps3.getButtonState(PS3::maru) || sw_tray_ue){
            f_Right.flag = 0;
        } else if ( ( sw_arm_hori_out && !sw_tray_ue ) || ( ps3.getButtonState(PS3::batu) && !sw_tray_ue ) ){
            f_Right.flag = -1;
        }

        if ( ps3.getButtonState(PS3::sankaku) ){
            f_Left.flag = 1;
        } else if ( ps3.getButtonState(PS3::maru) || sw_tray_sita){
            f_Left.flag = 0;
        } else if ( ( sw_arm_hori_out && !sw_tray_sita ) || ( ps3.getButtonState(PS3::batu) && !sw_tray_sita ) ){
            f_Left.flag = -1;
        }

        if ( ps3.getButtonState(PS3::R1) ){ // 右のトレーだけうごかせるよ
            send(M_R_TRAY, 0xff);
        } else if( ps3.getButtonState(PS3::R2) && sw_tray_ue ){
            send(M_R_TRAY, 0x80);
        } else if( ps3.getButtonState(PS3::R2) && !sw_tray_ue ){
            send(M_R_TRAY, 0x00);
        } else if( f_Right.flag == 0 ){
            send(M_R_TRAY, 0x80);
        } else if( f_Right.flag == 1 ){
            // 上にトレーが動いていくよ
            // send(M_L_TRAY, 0xff);
            send(M_R_TRAY, 0xff);
        } else if( f_Right.flag == -1 ){
            // リミットスイッチがどちらもおされていないとき、どちらも下にトレーが動いていくよ
            // send(M_L_TRAY, 0x00);
            send(M_R_TRAY, 0x00);
        }

        if ( ps3.getButtonState(PS3::L1) ){ // 左のトレーだけうごかせるよ
            send(M_L_TRAY, 0xff);
        } else if( ps3.getButtonState(PS3::L2) && sw_tray_sita ){
            send(M_L_TRAY, 0x80);
        } else if( ps3.getButtonState(PS3::L2) && !sw_tray_sita ){
            send(M_L_TRAY, 0x00);
        } else if( f_Left.flag == 0 ){
            send(M_L_TRAY, 0x80);
        } else if( f_Left.flag == 1 ){
            // 上にトレーが動いていくよ
            send(M_L_TRAY, 0xff);
            // send(M_R_TRAY, 0xff);
        } else if( f_Left.flag == -1 ){
            // リミットスイッチがどちらもおされていないとき、どちらも下にトレーが動いていくよ
            send(M_L_TRAY, 0x00);
            // send(M_R_TRAY, 0x00);
        }

        // printf("sw1:%d sw2:%d sw3:%d sw4:%d sw5:%d user:%d\n", sw_tray_ue, sw_tray_sita, sw_arm_ue, sw_arm_hori_in, sw_arm_hori_out, user_button);
        ThisThread::sleep_for(50ms);

    }

}

int thread1 (void){ // デバック用
    while (true){
        // ps3.printdata();
        // printf("lx:%2d ly:%2d la:%3.1f rx:%2d ry:%2d ra:%3.1f\n", ps3.getLeftJoystickXaxis(), ps3.getLeftJoystickYaxis(), ps3.getLeftJoystickAngle(), ps3.getRightJoystickXaxis(), ps3.getRightJoystickYaxis(), ps3.getRightJoystickAngle());
        // printf("Ly:%2d Ldata16:%2x Ldata10:%3d Ry:%2d Rdata16:%2x Rdata10:%3d \n", ps3.getLeftJoystickYaxis(), L_W_Mdata, L_W_Mdata, ps3.getRightJoystickYaxis(),R_W_Mdata, R_W_Mdata);
        // printf("sw1:%d sw2:%d sw3:%d sw4:%d sw5:%d user:%d\n", sw_tray_ue, sw_tray_sita, sw_arm_ue, sw_arm_hori_in, sw_arm_hori_out, user_button);
        ThisThread::sleep_for(100ms);
    }
}

// センサーの値を変数に代入するThread用関数
int thread2 (void){
    while (true){
        /*
        pulse1 = rori1.getPulses();
        pulse2 = rori2.getPulses();
        pulse3 = rori3.getPulses();
        pulse4 = rori4.getPulses();
        pulse5 = rori5.getPulses();
        pulse6 = rori6.getPulses();
        
        angle1 = ( 360*(float)pulse1 / (2048*2) );
        angle2 = ( 360*(float)pulse2 / (2048*2) );
        angle3 = ( 360*(float)pulse3 / (2048*2) );
        angle4 = ( 360*(float)pulse4 / (2048*2) );
        angle5 = ( 360*(float)pulse5 / (2048*2) );
        angle6 = ( 360*(float)pulse6 / (2048*2) );
        */
    
        // ポーリング用
        sw_tray_ue = sw1;
        sw_tray_sita = sw2;
        sw_arm_ue = sw3;
        sw_arm_hori_in = sw4;
        sw_arm_hori_out = sw5;
        user_button = button;

        myled = !button;
        led1 = sw1;
        led2 = sw2;
        led3 = sw3;
        led4 = sw4;
        led5 = sw5;

    }
}


void send (char address, char data){
    i2c.start();
    i2c.write(address);
    i2c.write(data);
    i2c.stop();
    ThisThread::sleep_for(1ms);
}

int outputMotorData_Arm_hori (char *data){
    int Rx;
    Rx = ps3.getRightJoystickXaxis();
    if ( Rx > 32 && !sw_arm_hori_in) {
        Rx = 32;
    } else if ( Rx < -32 && !sw_arm_hori_out ) {
        Rx = -32;
    } else {
        Rx = 0;
    }
    *data = 128 + Rx;
    return 0;
}

int outputMotorData_R_W (char *data){
    int Rx, Ry, sum;
    if ( ps3.getLeftJoystickYaxis() == 64){
        Ry = 64-1;
    }else{
        Ry = ps3.getLeftJoystickYaxis();
    }
    if ( ps3.getLeftJoystickXaxis() == -64){
        Rx = -63;
    }else{
        Rx = ps3.getLeftJoystickXaxis();
    }
    if ( Ry < 0 ){
        Rx = -1*Rx;
    }
    sum = Ry * 2 - Rx;
    if ( sum > 127 ){
        sum = 127;
    } else if ( sum < -127 ){
        sum = -127;
    }
    *data = 128 + sum;
    return 0;
}

int outputMotorData_L_W (char *data){
    int Rx, Ry, sum;
    if ( ps3.getLeftJoystickYaxis() == 64){
        Ry = 64-1;
    }else{
        Ry = ps3.getLeftJoystickYaxis();
    }
    if ( ps3.getLeftJoystickXaxis() == -64){
        Rx = -63;
    }else{
        Rx = ps3.getLeftJoystickXaxis();
    }
    if ( Ry < 0 ){
        Rx = -1*Rx;
    }
    sum = Ry * 2 + Rx;
    if ( sum > 127 ){
        sum = 127;
    } else if ( sum < -127 ){
        sum = -127;
    }
    *data = 128 + sum;
    return 0;
}

/*
// 下記関数はリミットスイッチの信号による割り込みで実行される関数
void sw_tray_sita_func () {
    if(sw1.read()){
        send(M_L_TRAY, 0x80);
    }
    sw_tray_sita = sw1.read();
    led1 = sw_tray_sita;
}

void sw_tray_ue_func () {
    if(sw2.read()){
        send(M_R_TRAY, 0x80);
    }
    sw_tray_ue = sw2.read();
    led2 = sw_tray_ue;
}

void sw_arm_ue_func () {
    if(sw_arm_ue){
        send(M_VERT_ARM, 0x80);
    }
    led3 = sw_arm_ue;
}

void sw_arm_hori_in_func () {
    if(sw4.read()){
        send(M_HORI_ARM, 0x80);
    }
    sw_arm_hori_in = sw4.read();
    led4 = sw_arm_hori_in;
}

void sw_arm_hori_out_func () {
    if(sw5.read()){
        send(M_HORI_ARM, 0x80);
    }
    sw_arm_hori_out = sw5.read();
    led5 = sw_arm_hori_out;
}
*/
