/***************************************************
2023発明。chatty_task_board （おしゃべりなタスクボード）
/***************************************************
初期Ver.
＜DFPlayer＞ 初期化が遅くてエラーになる問題をwhileで対策（初期化成功までループ）。電源不安定は別で対策。
<人感センサー> 人感センサ検知で読み上げを開始するが、人感センサが0になるまでリピートし続ける問題があった。
<割り込み> 旧Check関数で取得したTASK1～4をメインループに反映させるために、変数にvolatileを付けた。
これだけでは解決ぜす、⇒ 割り込み処理内で変数を変更するため、安全にやり取りするために割り込みを一時的に無効化する必要があった。
Ver.3.4
  myDFPlayer.play(10);  //()内はSDカード内での保存日時順）……という書き方を
  myDFPlayer.playFolder(1, 2); に変えることで、フォルダ分けでmp3ファイルを管理できるようになった。
Ver.3.5
  RTCへの対応試行中。RTCモジュールが、SDAとSCL用にアナログ4,5を使うので、それに合わせて他のピンを変更。
  テスト用（スロット1～4の読み取り値、電圧、抵抗のプリント）のコード短縮に成功。
  //TASK1だけ、なぜかたまに初回だけカード無し判定になる。原因不明。★★ ここに入るとその後ループが止まる。
Ver.3.6
　ピン配置を全面的に見直し。（ボードの"つまみ"を判定するリードスイッチの追加や、モータードライブICの追加などによる。）
  Check()を、CARD_Check()に変更。
Ver.3.6.1
  人感センサにmilis関数を導入。
Ver.3.6.2
  カード判定の共通処理をまとめるために関数を導入。
Ver.3.6.3
  Toggle_reset()関数を追加（電磁石でつまみを戻す。）
  TASK_Check()をToggle_Check()に名称変更。
Ver.3.6.4(正式)
  REC_time_check()関数を追加。(録音モジュール対応)
  REC長押し時間を取得し、Play_ISD1820()の時間に反映。
  ★ただし、解決不可能な問題あり。pressDurationの値は、電源を切ると消える。⇒初期値を2秒にして暫定対策。
  REC長押し中に人感センサトリガーからの動作を禁止　⇒　成功
　★★  読み上げ中にRECボタンを押したらどうなるかは未確認。
Ver.3.7
  read aloud1() 読み上げバターン1を関数化。
  Toggle_Goodjob() を追加。（いずれかのリードスイッチがオフになったらグッジョブ！）
Ver.3.7.2
  ランダムに褒めるようにした。（音声ファイル7～10）
Ver.3.7.3
  RTCに対応完了！
  カード読み取り結果のシリアルモニタ出力を、見やすい形に修正。
  設定ボタンを押した後、ループが止まる不具合を修正。原因は割り込み処理中に参照した関数にDelayがあったこと。カードチェック関数の中身を2つに分けて解決。
  設定ボタンを押して設定をリセットしたときに、人感センサが2週目以降のままなのを修正。
Ver.3.8
  一旦完成。2023/8/20
  pin3: Arduinoのリセットボタンを追加。
  DRV8835の出力を止め忘れていたのを修正。
ver.3.8.1
  DFPlayer が初回起動に失敗する問題を解決。直接の原因はおそらく電源が安定しないこと。
  初期化に成功するまでループするwhile文でDelayを100にした設定で、PCでは安定して成功していたが、  9V電池で動かすと失敗していた。
  その状態で、スイッチをONのままArduinoのリセットボタンを押すと正常に初期化できることから、瞬間的な電圧降下などの問題と考えられる。
  そこで、初期化ループのwhileの後、起動失敗のif文で、" asm volatile ("  jmp 0"); " Arduinoをリセット（0行まで戻す）。
  （起動失敗しなくなったので、もう緑のボタンは不要になった？）
Ver.3.8.2
  緑ボタンをなくし、DRV8835の2台目を追加。（1台で電磁石1,2独立、2台目で電磁石3,4を1ラインで操作）
  読み上げ2周目の「と、」を無しにした。
Ver.4.0 (Final)
  完成版。基本的にVer.3.8.2と同じ。印刷用に体裁を整え、不要な記述を削除する。
Ver.4.1（真の完成）
  電磁石の駆動時間などを微調整。
  "【最終防衛ライン】オフを挟まず、トグルが動くまで通電しっぱなし" を追加。
Ver.4.1_EN ◎（2024/5/25 世界展用に英語版カードを追加など。作成中……）

****************************************************/

#include "Arduino.h"
#include "SoftwareSerial.h"  //DFPlayerが使う。
#include "DFRobotDFPlayerMini.h"  //MP3プレイヤー用
#include "RTClib.h"  //RTCモジュール用

//----接続するArduinoソケットの番号----------------
int Check_BUTTON = 2; //設定ボタン。[割込]を使用　(旧:digitalPinToInterrupt(2) ）
//int Reset_BUTTON = 3; //リセットボタン（DFPleyerが起動失敗したとき or デモ用。[割込]を使用するまでもない？

int DFP_TX = 4;      // DFPlayer, TX端子。（旧：SoftwareSerial mySoftwareSerial(10, 11); //TX, RX（並び順を修正）
int DFP_RX = 5;      // DFPlayer, RX端子。
int BUSY_SOCKET = 6; // DFPlayer, Busy端子。再生中はLOWになる。

int Toggle1 = 7;  //タスクボード側のつまみ（リードスイッチ）のピン配置
int Toggle2 = 8;
int Toggle3 = 9;
int Toggle4 = 10;

int EM1 = 11;  //電磁石(1)を付けたモータードライブIC（DRV8835）1のAIN1
int EM2 = 12;  //電磁石(2)を付けたモータードライブIC（DRV8835）1のBIN1
int EM3 = 3;  //電磁石(3,4)を付けたモータードライブIC（DRV8835）2のAIN1  2023/8/26追加

int REC_BUTTON = 13;  // ISD1820のREC端子に接続されたタクトスイッチ。

const int SLOT1_pin = A0;  // 抵抗測定器。カードスロット1の読み取りピンをA0 (14)に固定
const int SLOT2_pin = A1;  // 　　　　　　　　　〃　　　2　　　〃　　　　A1 (15)に固定
const int SLOT3_pin = A2;  // 　　　　　　　　　〃　　　3　　　〃　　　　A2 (16)に固定
const int SLOT4_pin = A3;  // 　　　　　　　　　〃　　　4　　　〃　　　　A3 (17)に固定
                           //(18) = RTCのSDAとして使用。
                           //(19) = RTCのSCLとして使用。
//int EM3 = 18;  //電磁石2つを1ラインで動かすと磁力が弱い場合、RTCを無しにして電磁石(3) → DRV8835の2台目_AIN1
//int EM4 = 19;  //          〃                                           電磁石(4) → DRV8835の2台目_AIN1

const int JINKAN_pin =A6;  //Pin 20（アナログ）= 人感センサーの入力ピン。

int ISD1820_PlayPin = 21;  //ISD1820のP-E端子に再生命令を送るピン。
//----ピン番号ここまで-------------

/****** タスクスイッチ関連。2023/8/22修正 ******/
const unsigned long debounceDelay = 200;  // チャタリング防止のデバウンス時間。⇒200に。青、緑共通。
const unsigned long debounceDelay_Red = 100;  // チャタリング防止のデバウンス時間。赤ボタン。

volatile unsigned long lastInterruptTime_Blue = 0;  //青（確認）割り込みボタンのチャタリング防止用
volatile unsigned long lastInterruptTime_Green = 0;  //緑（リセット）割り込みボタンのチャタリング防止用
volatile unsigned long last_RecButtonTime = 0;  //RECボタン用。チャタリング防止用

/****** NEW !  チャタリング防止3（設定ボタン_loop内） **********************廃止*/ 
//volatile unsigned long last_Check_ButtonTime = 0;  // 青（確認）チャタリング防止用

volatile int Blue_Interrupt_Flag = false;  //設定ボタンの割り込みがあったことを示すフラグ
//volatile int Green_Interrupt_Flag = false;  //リセットボタンの割り込みがあったことを示すフラグ

int SLOT1_Analog;  //SLOT1で読み取ったアナログ値を格納
int SLOT2_Analog;  // 〃 2　　　〃
int SLOT3_Analog;  // 〃 3　　　〃
int SLOT4_Analog;  // 〃 4　　　〃

//読み取ったAnalog値からカードの種類を判定するためのAnalogRead閾値。
int CARD0 = 958;  //～無限大 Ω。この値以上ならカード未挿入。
int CARD1 = 830;  //6800 Ω。宿題
int CARD2 = 725;  //3000 Ω。水やり 旧691
int CARD3 = 649;  //2000 Ω。◎EN Homework
int CARD4 = 586;  //1500 Ω。お風呂掃除 旧538 → 575(card5が1100Ωのとき) → 586
int CARD5 = 510;  //1200 Ω。◎EN Watering 旧 1100 Ωのとき499
int CARD6 = 394;  // 820 Ω。明日の準備
int CARD7 = 291;  // 470 Ω。筋トレ 旧256
int CARD8 = 219;  // 330 Ω。◎EN Bathroom cleaning
int CARD9 = 115;  // 220 Ω。ピアノ練習 旧115
int CARD10 = 23;   //  47 Ω。録音カード。
//カード7は、わざわさ判定しなくても消去法で定まると考えていたが、カード未挿入時に0が返る場合があったため、切り分けのために必要。

volatile int TASK1=0;  //スロット1に入ったカードの判定結果（0～10）を記憶し、グローバル変数で扱う。初期値は0。
volatile int TASK2=0;  //スロット2　　〃
volatile int TASK3=0;  //スロット3　　〃
volatile int TASK4=0;  //スロット4　　〃

/************************************************************
電圧・抵抗測定。テスト用（シリアルモニタで確認）本番では無くてよい。
Analog入力をVに換算し、オームの法則から抵抗値を求める。*/
int Vcc = 5;               // 入力電圧5V 
int Rt = 1000;             // 基準抵抗(今回1kΩの抵抗を使用)

const int numSlots = 4;  //（コード短縮版）（forループ4回）
int analogPins[numSlots] = {A0, A1, A2, A3}; // テスト用（スケッチ短縮版）各スロットに対応するアナログピン
float Vx[numSlots];    //  それぞれのピンにおいて求めたい電圧Vx 
float Rx[numSlots];    //  それぞれのピンにおいて求めたい抵抗Rx

/****タスクボード通常動作のための設定**************************/
SoftwareSerial mySoftwareSerial(DFP_TX, DFP_RX); // RX, TX （TXとRXは互い違いに接続することに注意。）
DFRobotDFPlayerMini myDFPlayer;

volatile int JINKAN_Analog = 0;  //人感センサーからの読み取り値。初期値は0。volatile は特に関係なさそう。

/****チェックつまみ（トグル）の状態。1=未完了、0＝完了（リードスイッチON）。グローバル変数で扱う。
　　　初期値はこのタイミングで拾っておく。*/
volatile int current_TASK_state1 = digitalRead(Toggle1);  //スロット1段目の現在のタスク状態
volatile int current_TASK_state2 = digitalRead(Toggle2);
volatile int current_TASK_state3 = digitalRead(Toggle3);
volatile int current_TASK_state4 = digitalRead(Toggle4);

volatile int last_TASK_state1 = false;  //スロット1段目の前回のタスク状態 （定義追加2023/8/20）
volatile int last_TASK_state2 = false;
volatile int last_TASK_state3 = false;
volatile int last_TASK_state4 = false;

volatile int all_TASK_state_OFF = false;  //全てのタスクが完了（定義追加2023/8/20）

/*直近で人感センサが反応していた場合に、しばらく同じ動作をさせないための設定*/
unsigned long prevDetectionTime = 30000;  // 前回の人感センサーの検知時刻を保持する変数。
                //初期値0だと、初回の反応が遅いので、数字を大きくしておく（Replay_intervalの長さを目安に）。
const int JINKAN_THRESHOLD = 600;  // 人感センサーの閾値。人の検知は3V程度＝615前後。
int Replay_interval = 20000;  //再生を禁止する時間 (30秒)

volatile int JINKAN_count = 0; // 人感センサー検知による読み上げが2回目以降のときに、パターンを変えるための変数。

/***DRV8835（電磁石、ボードのつまみ）関係**********************/
int EM1_out=255;  //電磁石1の出力。（最大255）
int EM2_out=255;  //電磁石2の出力。（最大255）
int EM3_out=255;  //電磁石3,4並列の出力。（最大255）

/** RECボタンの処理用 *****************************************/
int RecButtonState = LOW;  //RECボタンの初期値
unsigned long pressStartTime = 0;
unsigned long pressDuration = 2000;  // ★解決不能問題あり。電源を切ると録音時間の値が失われるため、初期値を2秒にする。
bool isRecording = false;  // 録音中かどうかのフラグ ver.3.6.4で追加。

/*** RTCの、setup()前項目 18ピン = RTCのSDA。19ピン = RTCのSCL。*****************/
RTC_PCF8523 rtc;

const int targetHour = 7;  // ターゲット時刻（時）
const int targetMinute = 0;  // ターゲット時刻（分）
  // ※ループが1秒以上かかると処理が抜けるので、秒を指定しない。
const unsigned long executionDuration = 10000;  // 実行時間（ミリ秒）

bool executedToday = false;  // 今日の実行フラグ
DateTime lastExecution;  // 前回実行した日時

/************************************************************/

void setup() {

  //ピンモード設定
  pinMode(Check_BUTTON,INPUT_PULLUP);  //(2) [割込] 設定/設定ボタン。プルアップ付きの入力ピンとして設定する（押すとLOW）
  //pinMode(Reset_BUTTON,INPUT_PULLUP);   //(3) デモ用ボタン。プルアップ付きの入力ピンとして設定する（押すとLOW）
  pinMode(BUSY_SOCKET,INPUT);          //(6) DFPlayerのBusy端子を監視

  pinMode(Toggle1, INPUT_PULLUP);  //(7) タスクボード側のチェックつまみToggle（リードスイッチ）をプルアップ入力に設定。
  pinMode(Toggle2, INPUT_PULLUP);  //(8)
  pinMode(Toggle3, INPUT_PULLUP);  //(9)
  pinMode(Toggle4, INPUT_PULLUP);  //(10)

  pinMode(EM1, OUTPUT);  //(11) モータードライブIC_1のAIN1へ。
  pinMode(EM2, OUTPUT);  //(12) モータードライブIC_1のBIN1へ。
  pinMode(EM3, OUTPUT);  //(3) モータードライブIC_2のAIN1へ。
  
  pinMode(REC_BUTTON, INPUT);  //(13) 録音ボタンの長押し時間を監視。ISD1820のREC端子には別途スイッチを設け、GNDと抵抗の間で並列接続する。
  //アナログピンはpinMade しない。（14～17(A0～3), 20(A6))
  pinMode( ISD1820_PlayPin, OUTPUT );       //(21) ISD1820の再生端子につないだピンを出力に設定。

  //設定ボタン（青）を押したときの割り込み動作用。
  attachInterrupt(digitalPinToInterrupt(Check_BUTTON),BLUE_BUTTON,FALLING);  //LOWに変化した瞬間に関数BLUE_BUTTON()を実行するように割り込む。
  //attachInterrupt(digitalPinToInterrupt(Reset_BUTTON),Green_BUTTON,FALLING);  //LOWに変化した瞬間に関数Green_BUTTON()を実行するように割り込む。

  //---DFPlayer miniのセットアップ---------------------
  mySoftwareSerial.begin(9600);  //ソフトウェアシリアル通信の初期化と開始。DFPlayerの初期化。
  
  Serial.begin(9600);  //115200はDFPlayerのシリアル通信の速度。シリアルモニタが文字化けするときは、シリアルモニタの通信速度を合わせる。
  // DFPlayerの起動に失敗する原因を探るために、シリアルの速度を9600に変えてみた。
 
  Serial.println();
  Serial.println(F("DFRobot DFPlayer Mini Demo"));
  Serial.println(F("Initializing DFPlayer ... (May take 3~5 seconds)"));
  
  //★反応が返るまでループさせる。DFPlayerの初期化が遅くてエラーすることへの対策。
  while(!myDFPlayer.begin(mySoftwareSerial)){
    Serial.print(".");
    delay(100);  // 500→400→200→700→100
  }

  if (!myDFPlayer.begin(mySoftwareSerial)) {  //Use softwareSerial to communicate with mp3.
    // DFPlayerを初期化します。2秒以内に初期化できなかった場合はエラーメッセージを表示
    Serial.println(F("Unable to begin:"));
    Serial.println(F("1.Please recheck the connection!"));
    Serial.println(F("2.Please insert the SD card!"));

    asm volatile ("  jmp 0"); // リセット命令を実行してArduinoをリセット  ★！！最終的のこれで解決した。
    
    while(true){
      delay(0); // Code to compatible with ESP8266 watch dog.
    }
  }
  Serial.println(F("DFPlayer Mini online."));
  //delay(200);  //初期化後、たまにTimeOutするのを防止するため ←意味なさそう

  myDFPlayer.volume(22);  //DFPlayerの音量= 0 ～ 30

  //** RTCの項目************************************************
  if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    while (1);
  }

  if (! rtc.initialized() || rtc.lostPower()) {
    Serial.println("RTC is NOT initialized!");
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));  // 現在の日時でRTCを初期化
    //RTCモジュールが既に初期化されている場合は、rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));の行をコメントアウトすることで初期化をスキップできる。
  }

  lastExecution = rtc.now();

  //************************************************************

  //起動時に1回だけ、下記関数を実行。
  Play_MP3(1, 1);  //001_設定をリセットして～ 再生。
  CARD_Check();  //挿入されたカードを読み取り、記憶。
  Toggle_Check();  //つまみの状態を確認し、記憶する。
  Toggle_reset();  // つまみを戻す。
  Toggle_Check();  //つまみの状態を確認し、記憶する。

}

void loop() {

  /******** 設定ボタン Check_BUTTON（青）の処理 2023/8/22 ***************************
    割り込み中にDelayを使うとArduinoが止まる。この現象は、参照先の関数内でDerayが使われている場合でも同じだった。
    割り込み関数内では、フラグだけを取ってきて、loop内でPlay_MP3(1,1)を1回だけ鳴らす。
  */
  //if (Blue_Interrupt_Flag = true) {  //ミス。= trueを==trueにする。or trueは既にブール値なので、if (Blue_Interrupt_Flag)と書く。
  if (Blue_Interrupt_Flag) {  //ミス。= trueを==trueにする。or trueは既にブール値なので、if (Blue_Interrupt_Flag)と書く。

    // 青いボタンの処理*******************************
    Blue_Interrupt_Flag = false;
    Serial.println("Blue_Interrupt_Flag をONからオフへ");

    Play_MP3(1, 1);  //001_設定をリセットして、タスクカードを読み込みます。
    Toggle_reset();  //電磁石でつまみを元に戻す。
    CARD_Check();  //設定をリセット～の処理
    all_TASK_state_OFF = false;  //いきなりミッションコンプリートと言わないように。
    prevDetectionTime = 30000;  // 前回の人感センサーの検知時刻。すぐ開始できるようにしておく。
    JINKAN_count = 0;

  }

  /******** リセットボタン Reset_BUTTON（多分緑にする）の処理 2023/8/22 ******************/
  //if (Green_Interrupt_Flag) {
  //  Green_Interrupt_Flag = true;  //間違えてfalseにしていた
  //  Serial.println("Arduino をリセットします。");
  //  asm volatile ("  jmp 0"); // リセット命令を実行してArduinoをリセット
 //}

  /******** ISD1820と録音ボタンの処理 ********************************/

  RecButtonState = digitalRead(REC_BUTTON);  //REC端子の状態。1=ON, 0=OFF

  unsigned long currentMillis_Red = millis(); // チャタリング防止。 割り込み内でDelayを使えないためmillisを使用。
  if (currentMillis_Red - last_RecButtonTime >= debounceDelay_Red) {
  
    //if (RecButtonState == HIGH && pressStartTime == 0) {  // RECボタンが押された瞬間
    if (RecButtonState == HIGH && !isRecording) {
      isRecording = true;  // 録音中というフラグ
      pressStartTime = millis();
    //} else if (RecButtonState == LOW && pressStartTime != 0) {  // RECボタンが離された瞬間
    } else if (RecButtonState == LOW && isRecording) {
      isRecording = false;  // 録音終了フラグ
      pressDuration = millis() - pressStartTime;  //長押し時間を計算し、記憶する。
      pressStartTime = 0;  //←無いと正常に動作しない？
      Serial.print("REC長押し時間 (ミリ秒): ");
      Serial.println(pressDuration);
    }
  last_RecButtonTime = currentMillis_Red; 
  }

  /******** 録音中でないときの処理 *******************************************/
  
  if (!isRecording) {  // 録音中でない場合の動作。カッコ開始。

    int JINKAN_Analog = analogRead(JINKAN_pin); //人感センサーのアナログ値を読み取り。
    unsigned long currentTime = millis();  // 現在の時刻を取得

    // 人感センサーが人を検知し、一定時間が経過している場合
    if (JINKAN_Analog >= JINKAN_THRESHOLD && currentTime - prevDetectionTime >= Replay_interval) {
    
      prevDetectionTime = currentTime;  // 時刻を更新

      Serial.print(F("人感センサーが人を検知しました！　読み取り値＝"));
      Serial.println(JINKAN_Analog);

      if (JINKAN_count == 0) { // 初回の場合
        Read_aloud1();  //読み上げバターン1（初回）
        JINKAN_count++; // カウントを増やす

      } else { // 2回目以降の場合
        Read_aloud2();  //読み上げバターン1（2回目以降）
      }
    }
  }  // 録音中でない場合の動作。カッコ終わり。

  Toggle_Goodjob();

  //*** RTCの処理 *********************
  DateTime now = rtc.now();

  // 今日の実行がまだ行われておらず、ターゲット時刻に達した場合
  if (!executedToday && now.hour() == targetHour && now.minute() == targetMinute) {

    Play_MP3(1, 2); // おはよう！　今日のタスクを設定しよう。
    CARD_Check();  // 設定をリセットして、タスクを設定します。
    
    executedToday = true;
    lastExecution = now;
  }

  if (now > lastExecution) {  // 現在の日付が前回実行日付より後であれば、フラグをリセット
    executedToday = false;
  }
}

void Read_aloud1() {  // 読み上げ1（初回）***************************************

    Play_MP3(1, 3);  //タスクボードを確認してくれ
    delay(100);  //whileの前にdelay(100)が無いとうまくいかない。
    while(digitalRead(BUSY_SOCKET)==LOW){  //再生長さに対応
      delay(100);
    }
    Play_MP3(1, 4);  //今日のタスクは
    delay(100);
    while(digitalRead(BUSY_SOCKET)==LOW){
      delay(100);
    }

    switch (TASK1) {  //変数TASK1が一致するcaseを順に調べ、一致した次の行を実行
      case 1:
        Play_MP3(2, 1); // フォルダ02内のファイル001を再生(宿題)
        break;
      case 2:
        Play_MP3(2, 2); // 水やり
        break;
      case 3:
        Play_MP3(2, 3); // Homework
        break;
      case 4:
        Play_MP3(2, 4); // お風呂そうじ
        break;
      case 5:
        Play_MP3(2, 5); // Watering
        break;
      case 6:
        Play_MP3(2, 6); // 明日の準備
        break;
      case 7:
        Play_MP3(2, 7); // 筋トレ
        break;
      case 8:
        Play_MP3(2, 8); // Bathroom cleaning
        break;
      case 9:
        Play_MP3(2, 9); // ピアノ練習
        break;
      case 10:
        Play_ISD1820();  // 録音モジュールを再生
        break;
    }

    switch (TASK2) {  //変数TASK2が一致するcaseを順に調べ、一致した次の行を実行
      case 1:
        Play_MP3(1, 5); // と、
        Play_MP3(2, 1); //宿題
        break;
      case 2:
        Play_MP3(1, 5); // と、
        Play_MP3(2, 2); // 水やり
        break;
      case 3:
        Play_MP3(1, 5); // と、
        Play_MP3(2, 3); // Homework
        break;
      case 4:
        Play_MP3(1, 5); // と、
        Play_MP3(2, 4); // お風呂そうじ
        break;
      case 5:
        Play_MP3(1, 5); // と、
        Play_MP3(2, 5); // Watering
        break;
      case 6:
        Play_MP3(1, 5); // と、
        Play_MP3(2, 6); // 明日の準備
        break;
      case 7:
        Play_MP3(1, 5); // と、
        Play_MP3(2, 7); // 筋トレ
        break;
      case 8:
        Play_MP3(1, 5); // と、
        Play_MP3(2, 8); // Bathroom cleaning
        break;
      case 9:
        Play_MP3(1, 5); // と、
        Play_MP3(2, 9); // ピアノ練習
        break;
      case 10:
        Play_MP3(1, 5); // と、
        Play_ISD1820();  // 録音モジュールを再生
        break;
    }

    switch (TASK3) {  //変数TASK3が一致するcaseを順に調べ、一致した次の行を実行
      case 1:
        Play_MP3(1, 5); // と、
        Play_MP3(2, 1); //宿題
        break;
      case 2:
        Play_MP3(1, 5); // と、
        Play_MP3(2, 2); // 水やり
        break;
      case 3:
        Play_MP3(1, 5); // と、
        Play_MP3(2, 3); // Homework
        break;
      case 4:
        Play_MP3(1, 5); // と、
        Play_MP3(2, 4); // お風呂そうじ
        break;
      case 5:
        Play_MP3(1, 5); // と、
        Play_MP3(2, 5); // Watering
        break;
      case 6:
        Play_MP3(1, 5); // と、
        Play_MP3(2, 6); // 明日の準備
        break;
      case 7:
        Play_MP3(1, 5); // と、
        Play_MP3(2, 7); // 筋トレ
        break;
      case 8:
        Play_MP3(1, 5); // と、
        Play_MP3(2, 8); // Bathroom cleaning
        break;
      case 9:
        Play_MP3(1, 5); // と、
        Play_MP3(2, 9); // ピアノ練習
        break;
      case 10:
        Play_MP3(1, 5); // と、
        Play_ISD1820();  // 録音モジュールを再生
        break;
    }

    switch (TASK4) {  //変数TASK4が一致するcaseを順に調べ、一致した次の行を実行
      case 1:
        Play_MP3(1, 5); // と、
        Play_MP3(2, 1); //宿題
        break;
      case 2:
        Play_MP3(1, 5); // と、
        Play_MP3(2, 2); // 水やり
        break;
      case 3:
        Play_MP3(1, 5); // と、
        Play_MP3(2, 3); // Homework
        break;
      case 4:
        Play_MP3(1, 5); // と、
        Play_MP3(2, 4); // お風呂そうじ
        break;
      case 5:
        Play_MP3(1, 5); // と、
        Play_MP3(2, 5); // Watering
        break;
      case 6:
        Play_MP3(1, 5); // と、
        Play_MP3(2, 6); // 明日の準備
        break;
      case 7:
        Play_MP3(1, 5); // と、
        Play_MP3(2, 7); // 筋トレ
        break;
      case 8:
        Play_MP3(1, 5); // と、
        Play_MP3(2, 8); // Bathroom cleaning
        break;
      case 9:
        Play_MP3(1, 5); // と、
        Play_MP3(2, 9); // ピアノ練習
        break;
      case 10:
        Play_MP3(1, 5); // と、
        Play_ISD1820();  // 録音モジュールを再生
        break;
    }
    Play_MP3(1, 6); // 以上だ。よろしく頼む。
    delay(500);
  
}

void Read_aloud2() {  //読み上げ1（2回目以降）***********************************

  if (!all_TASK_state_OFF) {  // 全てのタスクが完了しているのでないならば

    current_TASK_state1 = digitalRead(Toggle1);
    current_TASK_state2 = digitalRead(Toggle2);
    current_TASK_state3 = digitalRead(Toggle3);
    current_TASK_state4 = digitalRead(Toggle4);

    Play_MP3(1, 12);  //おーい！
    delay(100);
    while(digitalRead(BUSY_SOCKET)==LOW){
      delay(100);
    }

    if (current_TASK_state1) {  //1段目のタスクが未完了なら

      switch (TASK1) {  //変数TASK1が一致するcaseを順に調べ、一致した次の行を実行
      case 1:
        Play_MP3(2, 1); // フォルダ02内のファイル001を再生(宿題)
        break;
      case 2:
        Play_MP3(2, 2); // 水やり
        break;
      case 3:
        Play_MP3(2, 3); // Homework
        break;
      case 4:
        Play_MP3(2, 4); // お風呂そうじ
        break;
      case 5:
        Play_MP3(2, 5); // Watering
        break;
      case 6:
        Play_MP3(2, 6); // 明日の準備
        break;
      case 7:
        Play_MP3(2, 7); // 筋トレ
        break;
      case 8:
        Play_MP3(2, 8); // Bathroom cleaning
        break;
      case 9:
        Play_MP3(2, 9); // ピアノ練習
        break;
      case 10:
        Play_ISD1820();  // 録音モジュールを再生
        break;
      }
    }
    
    if (current_TASK_state2) {
      switch (TASK2) {  //変数TASK2が一致するcaseを順に調べ、一致した次の行を実行
      case 1:
        //Play_MP3(1, 5); // と、
        Play_MP3(2, 1); //宿題
        break;
      case 2:
        //Play_MP3(1, 5); // と、
        Play_MP3(2, 2); // 水やり
        break;
      case 3:
        Play_MP3(2, 3); // Homework
        break;
      case 4:
        Play_MP3(2, 4); // お風呂そうじ
        break;
      case 5:
        Play_MP3(2, 5); // Watering
        break;
      case 6:
        Play_MP3(2, 6); // 明日の準備
        break;
      case 7:
        Play_MP3(2, 7); // 筋トレ
        break;
      case 8:
        Play_MP3(2, 8); // Bathroom cleaning
        break;
      case 9:
        Play_MP3(2, 9); // ピアノ練習
        break;
      case 10:
        Play_ISD1820();  // 録音モジュールを再生
        break;
      }
    }

    if (current_TASK_state3) {
      switch (TASK3) {  //変数TASK3が一致するcaseを順に調べ、一致した次の行を実行
      case 1:
        //Play_MP3(1, 5); // と、
        Play_MP3(2, 1); //宿題
        break;
      case 2:
        //Play_MP3(1, 5); // と、
        Play_MP3(2, 2); // 水やり
        break;
      case 3:
        Play_MP3(2, 3); // Homework
        break;
      case 4:
        Play_MP3(2, 4); // お風呂そうじ
        break;
      case 5:
        Play_MP3(2, 5); // Watering
        break;
      case 6:
        Play_MP3(2, 6); // 明日の準備
        break;
      case 7:
        Play_MP3(2, 7); // 筋トレ
        break;
      case 8:
        Play_MP3(2, 8); // Bathroom cleaning
        break;
      case 9:
        Play_MP3(2, 9); // ピアノ練習
        break;
      case 10:
        Play_ISD1820();  // 録音モジュールを再生
        break;
      }
    }
    if (current_TASK_state4) {
      switch (TASK4) {  //変数TASK4が一致するcaseを順に調べ、一致した次の行を実行
      case 1:
        //Play_MP3(1, 5); // と、
        Play_MP3(2, 1); //宿題
        break;
      case 2:
        //Play_MP3(1, 5); // と、
        Play_MP3(2, 2); // 水やり
        break;
      case 3:
        Play_MP3(2, 3); // Homework
        break;
      case 4:
        Play_MP3(2, 4); // お風呂そうじ
        break;
      case 5:
        Play_MP3(2, 5); // Watering
        break;
      case 6:
        Play_MP3(2, 6); // 明日の準備
        break;
      case 7:
        Play_MP3(2, 7); // 筋トレ
        break;
      case 8:
        Play_MP3(2, 8); // Bathroom cleaning
        break;
      case 9:
        Play_MP3(2, 9); // ピアノ練習
        break;
      case 10:
        Play_ISD1820();  // 録音モジュールを再生
        break;
      }
    }
  
    Play_MP3(1, 13);  // ～がまだ終わっていないぞ。
    delay(100);
    while(digitalRead(BUSY_SOCKET)==LOW){
      delay(100);
    }

  }  // 全てのタスクが完了しているのでないならば のカッコ
}

void Toggle_Goodjob() {  //グッジョブ or ミッション・コンプリート！

  // 各リードスイッチの状態を読み取る。
  current_TASK_state1 = digitalRead(Toggle1);
  current_TASK_state2 = digitalRead(Toggle2);
  current_TASK_state3 = digitalRead(Toggle3);
  current_TASK_state4 = digitalRead(Toggle4);

  // いずれかのリードスイッチが1から0に切り替わった場合 　2023/8/20追記
  if ((last_TASK_state1 == true && current_TASK_state1 == false) ||
      (last_TASK_state2 == true && current_TASK_state2 == false) ||
      (last_TASK_state3 == true && current_TASK_state3 == false) ||
      (last_TASK_state4 == true && current_TASK_state4 == false)) {
    
    //褒め言葉をランダム化。1グッジョブ、2よくできました、3よくできました父、4素晴らしいわ
    int praise_at_random = random(7, 11); // 7以上11未満の範囲のランダムな整数を生成
    Play_MP3(1, praise_at_random); // グッジョブ！

  }

  // 全てのリードスイッチがオフになった場合
  if (!current_TASK_state1 && !current_TASK_state2 &&
      !current_TASK_state3 && !current_TASK_state4) {
    if (!all_TASK_state_OFF) {

      Play_MP3(1, 11); // ミッション・コンプリート！
      
      all_TASK_state_OFF = true;
    }
  } else {
    all_TASK_state_OFF = false;
  }

  last_TASK_state1 = current_TASK_state1;  //現在の状態を保存
  last_TASK_state2 = current_TASK_state2;
  last_TASK_state3 = current_TASK_state3;
  last_TASK_state4 = current_TASK_state4;

}

void Play_ISD1820() {  // 【関数】ISD1820の再生
  digitalWrite(ISD1820_PlayPin, HIGH);
  delay(pressDuration);  //記憶した「LEC長押し時間」分だけ継続。
  delay(debounceDelay_Red);  //チャタリング防止のDelay分、再生時間待ち時間を長くする。
  digitalWrite(ISD1820_PlayPin, LOW);
}

void Play_MP3(int folder, int file) {  //【MP3を再生する関数】

  myDFPlayer.playFolder(folder, file);
  delay(200);  //  MP3(と、) が入った場合、短すぎるから不具合が出る。現在200。
  while (digitalRead(BUSY_SOCKET) == LOW) {
    delay(70);  //100から70に調整。
  }
}

void Toggle_reset() {  //電磁石でタスクボードのつまみ（トグル）をリセット。電池の急な電圧降下を避けるため、動作を分ける。

  analogWrite(EM1,EM1_out);  //電磁石1へ出力。
  delay(200);  //電磁石1個ずつONなら300でも十分。→ダメな場合があるので500に→ループがあるので最初は200
  analogWrite(EM1,0);  //電磁石1への出力を0にする。

  analogWrite(EM2,EM2_out);  //電磁石2へ出力。 巻き線きれいな方
  delay(200);
  analogWrite(EM2,0);  //電磁石2への出力を0にする。
  delay(20);  // 電圧降下からの回復するのを待つ。意味ある？

  analogWrite(EM3,EM3_out);  //電磁石3,4へ出力。 巻き線きれいな方
  delay(200);  //1000→700 長すぎるので(リトライもあるし) 2023/8/30
  analogWrite(EM3,0);  //電磁石3,4への出力を0にする。

  delay(20);
  current_TASK_state1 = digitalRead(Toggle1);  //更新
  current_TASK_state2 = digitalRead(Toggle2);
  current_TASK_state3 = digitalRead(Toggle3);
  current_TASK_state4 = digitalRead(Toggle4);

  //****【電磁石EM1 最終防衛ライン】オフを挟まず、トグルが動くまで通電しっぱなし。
  int Toggle_ResetCount1 = 0;  //***通電時間が長いと発熱するのでやめる。
  while (Toggle_ResetCount1 < 8) {  //8×200msで最大1600ms

    if (current_TASK_state1 == 0) {  //2023/8/30追加
      analogWrite(EM1,EM1_out);
      delay(200);
      Serial.print("EM1 最終防衛ライン　★リトライしました。while ループ数＝ ");
      Serial.println(Toggle_ResetCount1);
    }
    delay(20);
    current_TASK_state1 = digitalRead(Toggle1);  //トグルの状態を更新

    if (current_TASK_state1 == 1) {  //2023/8/30追加
      analogWrite(EM1,0);
      Serial.print("EM1 最終防衛ライン　クリアしました。while ループ数＝ ");
      Serial.println(Toggle_ResetCount1);
      Toggle_ResetCount1=8;
    }
    Toggle_ResetCount1++;
  }
  analogWrite(EM1,0);  //  無理だった場合、通電をオフにする。

  //****【電磁石EM2 最終防衛ライン】オフを挟まず、トグルが動くまで通電しっぱなし。
  int Toggle_ResetCount2 = 0;  //***通電時間が長いと発熱するのでやめる。
  while (Toggle_ResetCount2 < 8) {  //8×200msで最大1600ms

    if (current_TASK_state2 == 0) {  //2023/8/30追加
      analogWrite(EM2,EM2_out);
      delay(200);
      Serial.print("EM2 最終防衛ライン　★リトライしました。while ループ数＝ ");
      Serial.println(Toggle_ResetCount2);
    }
    delay(20);
    current_TASK_state2 = digitalRead(Toggle2);  //トグルの状態を更新

    if (current_TASK_state2 == 1) {  //2023/8/30追加
      analogWrite(EM2,0);
      Serial.print("EM2 最終防衛ライン　クリアしました。while ループ数＝ ");
      Serial.println(Toggle_ResetCount2);
      Toggle_ResetCount2=8;
    }
    Toggle_ResetCount2++;
  }
  analogWrite(EM2,0);  //  無理だった場合、通電をオフにする。

  //****【電磁石EM3 最終防衛ライン】オフを挟まず、トグルが動くまで通電しっぱなし。
  int Toggle_ResetCount3 = 0;  //***通電時間が長いと発熱するのでやめる。
  while (Toggle_ResetCount3 < 8) {  //8×200msで最大1600ms

    if (current_TASK_state3 == 0 || current_TASK_state4 == 0) {
      analogWrite(EM3,EM3_out);
      delay(200);
      Serial.print("EM3 最終防衛ライン　★リトライしました。while ループ数＝ ");
      Serial.println(Toggle_ResetCount3);
    }
    delay(20);
    current_TASK_state3 = digitalRead(Toggle3);  //トグルの状態を更新
    current_TASK_state4 = digitalRead(Toggle4);  //トグルの状態を更新

    if (current_TASK_state3 == 1 && current_TASK_state4 == 1) {  //ここは&&
      analogWrite(EM3,0);
      Serial.print("EM3 最終防衛ライン　クリアしました。while ループ数＝ ");
      Serial.println(Toggle_ResetCount3);
      Toggle_ResetCount3=8;
    }
    Toggle_ResetCount3++;
  }
  analogWrite(EM3,0);  //  無理だった場合、通電をオフにする。

}

void Toggle_Check() {  /**** 【関数】タスクボードの「完/未完」つまみをチェックする　*******/

  // 各リードスイッチの状態を読み取り、シリアルモニタに出力する。
  current_TASK_state1 = digitalRead(Toggle1);
  current_TASK_state2 = digitalRead(Toggle2);
  current_TASK_state3 = digitalRead(Toggle3);
  current_TASK_state4 = digitalRead(Toggle4);

  Serial.print("タスク1の状態:");  //プルアップにしているからなのか、磁石に反応していないときに「1」になる。
  Serial.print(current_TASK_state1);
  
  Serial.print("、タスク2の状態：");
  Serial.print(current_TASK_state2);
  
  Serial.print("、タスク3の状態：");
  Serial.print(current_TASK_state3);
  
  Serial.print("、タスク4の状態：");
  Serial.println(current_TASK_state4);

}

void BLUE_BUTTON() {  //青ボタン割り込み。CARD_Checkの中身を2つに分ける。
  noInterrupts();  // 割り込み処理内で安全に変数を変更するために、割り込みを無効化

  unsigned long currentMillis_Blue = millis(); // チャタリング防止1。 割り込み内でDelayを使えないためmillisを使用。 
  if (currentMillis_Blue - lastInterruptTime_Blue >= debounceDelay) {

  Blue_Interrupt_Flag = true;  //MP3(1,1)をloop内で再生するためのフラグ
  //Serial.println("Blue_Interrupt_Flag をONにした。");

  lastInterruptTime_Blue = currentMillis_Blue;  // チャタリング防止1。
  }
  interrupts(); // 割り込みを再度有効化
}

  /* ピン数節約のため廃止（EM3を導入）
void Green_BUTTON() {  //緑ボタン（リセット）割り込み。
  //noInterrupts();

  unsigned long currentMillis_Green = millis(); // チャタリング防止。 割り込み内でDelayを使えないためmillisを使用。 
  if (currentMillis_Green - lastInterruptTime_Green >= debounceDelay) {

    noInterrupts();

    Green_Interrupt_Flag = true;  //リセットのフラグを立ててループに戻る。
    Serial.println("緑のボタンを押した");

    interrupts(); // 割り込みを再度有効化  こっちにしたらリセットが効くようになった。

    lastInterruptTime_Green = currentMillis_Green;  // チャタリング防止
  }
  //interrupts(); // 割り込みを再度有効化
}
  */

void CARD_Check() {  /**【関数】設定をリセット（カード種類チェック）※割り込みボタン、RTCでも参照 ****/

  //Play_MP3(1, 1);  //001_設定をリセットして～ ←これが原因で止まっていた。内部でDelayが使われていたため。

  /******カード種類判定スケッチの短縮版************/
  int slotAnalog[numSlots];  
  int task[numSlots];

  for (int i = 0; i < numSlots; i++) {
    slotAnalog[i] = analogRead(analogPins[i]);
    task[i] = determineCardType(slotAnalog[i]);
      
    switch (i) {  //短縮版で、TASK1～4に、カード種別の番号を代入する。
      case 0: TASK1 = task[i]; break;
      case 1: TASK2 = task[i]; break;
      case 2: TASK3 = task[i]; break;
      case 3: TASK4 = task[i]; break;
    }

    Serial.print("SLOT");
    Serial.print(i + 1);
    Serial.print("のアナログ値: ");
    Serial.print(slotAnalog[i]);
    Serial.print("   電圧(Vx): ");
    Serial.print((slotAnalog[i] * Vcc) / 1024.0);
    Serial.print(" V");
    Serial.print("   ===>> ");
    Serial.print("   抵抗値：");
    Serial.print(calculateResistance((slotAnalog[i] * Vcc) / 1024.0));
    Serial.print(" Ω");

    printCardType(task[i]);  //追加 2023/8/17
  }
}

int determineCardType(int analogValue) {  //カード判定の共通処理をまとめるために関数を導入（カード種別の番号を返す）
  if (analogValue >= CARD0) return 0; //カードなし
  if (analogValue >= CARD1) return 1; //宿題
  if (analogValue >= CARD2) return 2; //水やり
  if (analogValue >= CARD3) return 3; //Homework ◎EN
  if (analogValue >= CARD4) return 4; //お風呂掃除
  if (analogValue >= CARD5) return 5; //Watering ◎EN
  if (analogValue >= CARD6) return 6; //明日の準備
  if (analogValue >= CARD7) return 7; //筋トレ
  if (analogValue >= CARD8) return 8; //Bathroom cleaning ◎EN
  if (analogValue >= CARD9) return 9; //ピアノ練習
  if (analogValue >= CARD10) return 10; //録音
  return 0;
}

float calculateResistance(float voltage) {  //カード判定の共通処理をまとめるために関数を導入(オームの法則計算)
  return voltage * Rt / (Vcc - voltage);
}

void printCardType(int taskValue) {  //カード判定の共通処理をまとめるために関数を導入（判定結果をprint）
  switch (taskValue) {
    case 0: Serial.println(" ==>> カードが挿入されていません。"); break;
    case 1: Serial.println(" ==>> カード1、6800 Ωです。"); break;
    case 2: Serial.println(" ==>> カード2、3000 Ωです。"); break;
    case 3: Serial.println(" ==>> カード3、2000 Ωです。"); break; //◎EN
    case 4: Serial.println(" ==>> カード4、1500 Ωです。"); break;
    case 5: Serial.println(" ==>> カード5、1100 Ωです。"); break; //◎EN
    case 6: Serial.println(" ==>> カード6、820 Ωです。"); break;
    case 7: Serial.println(" ==>> カード7、470 Ωです。"); break;
    case 8: Serial.println(" ==>> カード8、330 Ωです。"); break; //◎EN
    case 9: Serial.println(" ==>> カード9、220 Ωです。"); break;
    case 10: Serial.println(" ==>> カード10、47 Ωです。"); break; //録音
    default: Serial.println(" ==>> カードが挿入されていません。"); break;
  }
}