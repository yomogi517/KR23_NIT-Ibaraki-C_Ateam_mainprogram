// ジョイスティック1本のみ
#include "mbed.h"
#include "QEI.h"
#include "PS3.h"
#include <cstdint>  //uint8 (char) を使用したら勝手に追加された

#define M_R_WHEEL   0x40
#define M_L_WHEEL   0x42
#define M_L_TRAY    0x44    // 下のトレー
#define M_R_TRAY    0x46    // 上のトレー
#define M_VERT_ARM  0x50
#define M_HORI_ARM  0x52

Thread th1;
Thread th2; // エンコーダ用
int thread1();  // デバック用printfを100msごとにまわすためのThread
int thread2(); 

// 使用するピンおよび機能の宣言
I2C i2c(D14,D15); // D14 = PB_8, D15 = PB_9
PS3 ps3( A0, A1); // PA_0, PA_1

DigitalIn button(BUTTON1);  // Nucleo上にある青いボタン(デバック用)
// リミットスイッチ用DigitalIn(基板上でプルアップしているため、リミットスイッチの配線をNC,COMにつなぐと通常時０,押されたとき１、NO,COMにつなぐと通常時１,押されたとき０)
/*
DigitalIn sw1(PC_4);        // 昇降SW上
DigitalIn sw2(PB_15);       // 昇降SW下
DigitalIn sw3(PB_12);       // アーム上SW
DigitalIn sw4(PB_15);       // アーム左右SW1
DigitalIn sw5(PC_8);        // アーム左右sw2
*/
InterruptIn sw1(PC_4);        // 昇降SW下（左）
InterruptIn sw2(PB_15);       // 昇降SW上（右）
InterruptIn sw3(PB_12);       // アーム上SW
InterruptIn sw4(PB_15);       // アーム左右SW1機体内側(hori_in)
InterruptIn sw5(PC_8);        // アーム左右sw2機体外側(hori_out)

DigitalOut EMG(D13);    // 遠隔緊急停止
DigitalOut myled(LED1); // Nucleo上にあるチップLED
DigitalOut led1(PA_9);  //  PA_9 = D8
DigitalOut led2(PA_8);  //  PA_8 = D7
DigitalOut led3(PB_10); // PB_10 = D6
DigitalOut led4(PB_4);  //  PB_4 = D5
DigitalOut led5(PB_5);  //  PB_5 = D4

/* ロータリーエンコーダ_外部ライブラリQEIを使用(機体に積まないためコメントアウト)
QEI rori1(  PD_2, PC_11, NC, 2048, QEI::X2_ENCODING);
QEI rori2( PC_10, PC_12, NC, 2048, QEI::X2_ENCODING);
QEI rori3( PA_13, PA_14, NC, 2048, QEI::X2_ENCODING);
QEI rori4( PA_15,  PB_7, NC, 2048, QEI::X2_ENCODING);
QEI rori5(  PH_0,  PH_1, NC, 2048, QEI::X2_ENCODING);
QEI rori6(  PC_3,  PA_4, NC, 2048, QEI::X2_ENCODING);
*/

void send(char, char);
int outputMotorData_R_W (char *data);
int outputMotorData_L_W (char *data);
void sw_tray_sita_func ();
void sw_tray_ue_func ();
void sw_arm_ue_func ();
void sw_arm_hori_in_func ();
void sw_arm_hori_out_func ();

/* エンコーダ用変数
int pulse1, pulse2, pulse3, pulse4, pulse5, pulse6;
float angle1, angle2, angle3, angle4, angle5, angle6;
*/

char R_W_Mdata, L_W_Mdata; //足回りのMDに送るPWMデータ
int sw_tray_ue, sw_tray_sita, sw_arm_ue, sw_arm_hori_in, sw_arm_hori_out, user_button; // printf用変数

int main (void){

    sw1.rise(&sw_tray_sita_func);
    sw2.rise(&sw_tray_ue_func);
    sw3.rise(&sw_arm_ue_func);
    sw4.rise(&sw_arm_hori_in_func);
    sw5.rise(&sw_arm_hori_out_func);
    sw1.fall(&sw_tray_sita_func);
    sw2.fall(&sw_tray_ue_func);
    sw3.fall(&sw_arm_ue_func);
    sw4.fall(&sw_arm_hori_in_func);
    sw5.fall(&sw_arm_hori_out_func); 

    EMG = 0;

    // ショートブレーキ(break)状態へ
    R_W_Mdata = 0x80;
    L_W_Mdata = 0x80;

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
        send(M_R_WHEEL, L_W_Mdata);
        send(M_L_WHEEL, 256 - R_W_Mdata);

        if( ps3.getButtonState(PS3::migi) && !sw_arm_hori_out ){ // 動かしてみて、リミットスイッチの確認
            // send(M_HORI_ARM, 0xc0);     // 128+64の出力（半分くらい）
            send(M_HORI_ARM, 0xe0);        // 128+64+32 
        } else if ( ps3.getButtonState(PS3::hidari) && !sw_arm_hori_in){
            // send(M_HORI_ARM, 0x40);     // 128-64の出力（半分くらい）
            send(M_HORI_ARM, 0x20);        // 128-63-32
        } else {
            send(M_HORI_ARM, 0x80);     // break
        }

        if( ps3.getButtonState(PS3::ue) && !sw_arm_ue ){
            // send(M_VERT_ARM, 0x58);     // 128-40の出力（適当だがあまり電圧必要ないと判断）
            send(M_VERT_ARM, 0x44);        // 128-60
        } else if ( ps3.getButtonState(PS3::sita) ){
            // send(M_VERT_ARM, 0xa8);     // 128+40の出力（同上）
            send(M_VERT_ARM, 0xbc);        // 128+60
        } else {
            send(M_VERT_ARM, 0x80);     // break
        }

        if( ps3.getButtonState(PS3::sankaku) ){
            // 上にトレーが動いていくよ
            send(M_L_TRAY, 0xff);
            send(M_R_TRAY, 0xff);
        } else if( ps3.getButtonState(PS3::batu) && !sw_tray_sita && !sw_tray_ue ){
            // リミットスイッチがどちらもおされていないとき、どちらも下にトレーが動いていくよ
            send(M_L_TRAY, 0x00);
            send(M_R_TRAY, 0x00);
        } else if( ps3.getButtonState(PS3::batu) && sw_tray_sita && !sw_tray_ue){
            // リミットスイッチが下のトレー（左側）が押されているとき、上のトレー（右）しか下に動かないよ
            send(M_L_TRAY, 0x80);
            send(M_R_TRAY, 0x00);
        } else if( ps3.getButtonState(PS3::batu) && sw_tray_sita && sw_tray_ue){
            // リミットスイッチがどちらとも押されているとき、どちらも×を押しても下にうごかないよ
            send(M_L_TRAY, 0x80);
            send(M_R_TRAY, 0x80);
        } else {
            send(M_L_TRAY, 0x80);
            send(M_R_TRAY, 0x80);
        }

        ThisThread::sleep_for(50ms);

    }

}

int thread1 (void){
    while (true){
        // ps3.printdata();
        // printf("lx:%2d ly:%2d la:%3.1f rx:%2d ry:%2d ra:%3.1f\n", ps3.getLeftJoystickXaxis(), ps3.getLeftJoystickYaxis(), ps3.getLeftJoystickAngle(), ps3.getRightJoystickXaxis(), ps3.getRightJoystickYaxis(), ps3.getRightJoystickAngle());
        // printf("Ly:%2d Ldata16:%2x Ldata10:%3d Ry:%2d Rdata16:%2x Rdata10:%3d \n", ps3.getLeftJoystickYaxis(), L_W_Mdata, L_W_Mdata, ps3.getRightJoystickYaxis(),R_W_Mdata, R_W_Mdata);
        // printf("%d\n",Lx);
        printf("sw1:%d sw2:%d sw3:%d sw4:%d sw5:%d user:%d\n", sw_tray_ue, sw_tray_sita, sw_arm_ue, sw_arm_hori_in, sw_arm_hori_out, user_button);
        ThisThread::sleep_for(100ms);
    }
}

// エンコーダの値を変数に代入するThread用関数（今はリミットスイッチ）
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
        /*
        sw_tray_sita = sw1;
        sw_tray_ue = sw2;
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
        */

        /*
        if( sw1 || user_button ){
            led1 = 1;
        } else {
            led1 = 0;
        }

        if(sw2){
            led2 = 1;
        } else {
            led2 = 0;
        }

        if(sw3){
            led3 = 1;
        } else {
            led3 = 0;
        }

        if(sw4){
            led4 = 1;
        } else {
            led4 = 0;
        }

        if(sw5){
            led5 = 1;
        } else {
            led5 = 0;
        }
        */
        
    }
}


void send (char address, char data){
    i2c.start();
    i2c.write(address);
    i2c.write(data);
    i2c.stop();
    ThisThread::sleep_for(1ms);
    // wait_us(50000);
}

int outputMotorData_R_W (char *data){
    int Rx, Ry, sum;
    if ( ps3.getRightJoystickYaxis() == 64){
        Ry = 64-1;
    }else{
        Ry = ps3.getRightJoystickYaxis();
    }
    if ( ps3.getRightJoystickXaxis() == -64){
        Rx = -63;
    }else{
        Rx = ps3.getRightJoystickXaxis();
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
    if ( ps3.getRightJoystickYaxis() == 64){
        Ry = 64-1;
    }else{
        Ry = ps3.getRightJoystickYaxis();
    }
    if ( ps3.getRightJoystickXaxis() == -64){
        Rx = -63;
    }else{
        Rx = ps3.getRightJoystickXaxis();
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
    if(sw3.read()){
        send(M_VERT_ARM, 0x80);
    }
    sw_arm_ue = sw3.read();
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