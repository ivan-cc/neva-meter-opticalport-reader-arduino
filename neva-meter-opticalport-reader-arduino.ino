/*
  Data reader from the "Neva" (by Taipit) electricity meter through the optical port.
  Читальщик данных через оптический порт электросчетчика "Нева" (Тайпит).

  Author: Ivan
  https://github.com/ivan-cc/neva-meter-opticalport-reader-arduino
  I.2022

  Electricity meters: https://www.meters.taipit.ru/
 */
 
#include <SoftwareSerial.h>

//раскоментируйте это, если вы используете трех-фазный счётчик
//#define THREE_PHASE_METER 1

//port for RX
#define METERS_IN 2 

 //port for TX
#define METERS_OUT  4

 //светодиод, показывающий состояние коннекта со счетчиком
#define IS_CONNECTED_METER_LED_PIN 13

//раскоментируйте, если нужно выполнять запросы, пришедшие из uart
#define WAIT_REQUESTS_FROM_UART 1 

//пауза после запуска ардуино, мс
#define PAUSE_AFTER_START 500

//пауза после отправки и чтения запроса из счетчика, мс
#define PAUSE_AFTER_QUERY 100 

//следующий запрос параметров электросети делать через, сек
#define NEXT_ELECTRICITY_DATA_REQUEST_IN 30

//следующий запрос данных из счетчика делать через, сек
#define NEXT_METER_VALUES_REQUEST_IN 180

//следующий запрос информации о счетчике делать через, сек
#define NEXT_METER_REQUEST_IN 600

//переодичность пинга счетчика, сек (0 - чтобы не пинговать)
#define NEXT_PING_IN 7

//количество попыток подключиться к счетчику
#define NUMBER_OF_ATTEMPTS_TO_CONNECT 5

//пин выхода генератора-таймера (OCR1A)
#define OCR1A_OUT_PIN 9

//запрашивать ли расширенные параметры электросети (реактивную мощность, угол между фазами для трех-фазного счетчика)
#define EXTENDED_DATA_NEEDED true

//выводить данные без названия переменных
#define OUTPUT_WITHOUT_NAME false




/*** ДАЛЬШЕ СИСТЕМНЫЕ КОНСТАНТЫ ***/
//стандартная длинна строки запроса в счетчик
#define REQUEST_STRING_LENGTH 16

//длинна строки с короткой записью запроса (не полной, например 01020F)
#define SHORT_REQUEST_LENGTH 6

//скорость порта для контроля. Serial
#define SERIAL_SPEED 9600

//UART speed for init
#define INIT_SPEED 300 

//UART speed for request data
#define BASE_SPEED 9600 



SoftwareSerial MeterSerial(METERS_IN, METERS_OUT, false ); // RX, TX, inverse

//счетчики для отсчета запросов
int COUNTER_electricity_data = 0;
int COUNTER_meter_values = 0;
int COUNTER_meter_info = 0;
char COUNTER_ping = NEXT_PING_IN;

String outputString = "";
int incomingByte = 0;

char iter=0;

#ifdef WAIT_REQUESTS_FROM_UART
  String inputStringFromSerial = "";
  char inChar;
  char inputShort[SHORT_REQUEST_LENGTH];
#endif

//признак успешного соединения со счетчиком
bool isConnected = false;


/* ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
 *        Requests
 *        Запросы
 * ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
 */
/*
 * Запросы для инициализации
 */
//инициализация
// /?!
byte Init[] = {0x2F,0x3F,0x21,0x0D,0x0A};
//подтверждение на изменение скорости
// .051
byte OkAndChangeSpeed[] = {0x06, 0x30, 0x35, 0x31, 0x0D, 0x0A};
//pass
// .P1.(00000000)..
byte Pass[] = {0x01,0x50,0x31,0x02,0x28,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x29,0x03,0x61};


/*
*    Текущие параметры электросети 
*/
// - - - текущее напряжение в сети
#ifdef THREE_PHASE_METER
  byte VoltageP1[] = "200700";//{0x01,0x52,0x31,0x02,0x32,0x30,0x30,0x37,0x30,0x30,0x46,0x46,0x28,0x29,0x03,0x66}; 
  byte VoltageP2[] = "340700";//{0x01,0x52,0x31,0x02,0x33,0x34,0x30,0x37,0x30,0x30,0x46,0x46,0x28,0x29,0x03,0x63}; 
  byte VoltageP3[] = "480700";//{0x01,0x52,0x31,0x02,0x34,0x38,0x30,0x37,0x30,0x30,0x46,0x46,0x28,0x29,0x03,0x68};
#else
  //.R1.0C0700FF()..
  byte Voltage[] = "0C0700";//{0x01,0x52,0x31,0x02,0x30,0x43,0x30,0x37,0x30,0x30,0x46,0x46,0x28,0x29,0x03,0x17};
#endif
//--

// - - - текущий ток через счетчик
#ifdef THREE_PHASE_METER
  byte CurrentP1[]="1F0700";//{0x01,0x52,0x31,0x02,0x31,0x46,0x30,0x37,0x30,0x30,0x46,0x46,0x28,0x29,0x03,0x13};
  byte CurrentP2[]="330700";//{0x01,0x52,0x31,0x02,0x33,0x33,0x30,0x37,0x30,0x30,0x46,0x46,0x28,0x29,0x03,0x64};
  byte CurrentP3[]="470700";//{0x01,0x52,0x31,0x02,0x34,0x37,0x30,0x37,0x30,0x30,0x46,0x46,0x28,0x29,0x03,0x67};
#else
  byte Current[]="0B0700";//{0x01,0x52,0x31,0x02,0x30,0x42,0x30,0x37,0x30,0x30,0x46,0x46,0x28,0x29,0x03,0x16};
#endif

// - - - текущая мощность через счетчик
#ifdef THREE_PHASE_METER
  byte APowerSum[]="100700";
  byte APowerP1[]="240700";
  byte APowerP2[]="380700";
  byte APowerP3[]="4C0700";

  byte RPowerPositiveSum[] = "030701";
  byte RPowerPositiveP1[]="170701";
  byte RPowerPositiveP2[]="2B0701";
  byte RPowerPositiveP3[]="3F0701";

  byte RPowerNegativeSum[] = "040701";
  byte RPowerNegativeP1[]="180701";
  byte RPowerNegativeP2[]="2C0701";
  byte RPowerNegativeP3[]="400701"; 
#else
  byte APower[]="100700";//{0x01,0x52,0x31,0x02,0x31,0x30,0x30,0x37,0x30,0x30,0x46,0x46,0x28,0x29,0x03,0x65};

  byte RPowerPositive = "030701";
  byte RPowerNegative = "040701";
#endif

// - - - коэфициент активной мощности (cos)
#ifdef THREE_PHASE_METER
  byte PowerFactorSum[]="0D07FF";
  byte PowerFactorP1[]="2107FF";//{01 52 31 02 32 31 30 37 46 46 46 46 28 29 03 67};
  byte PowerFactorP2[]="3507FF";//{01 52 31 02 33 35 30 37 46 46 46 46 28 29 03 62};
  byte PowerFactorP3[]="4907FF";//{01 52 31 02 34 39 30 37 46 46 46 46 28 29 03 69};
#else
  //answer: NX.XX (N=2 cos=1;  N=1 индуктивная нагрузка;N=0 емкостная нагрузка)
  byte PowerFactor[]="0D07FF";//{0x01,0x52,0x31,0x02,0x31,0x30,0x30,0x37,0x30,0x30,0x46,0x46,0x28,0x29,0x03,0x65};
#endif

// - - - угол между фазами
#ifdef THREE_PHASE_METER
  byte AngleVoltageP1_P2[] = "51070A";
  byte AngleVoltageP2_P3[] = "510715";
  byte AngleVoltageP1_P3[] = "510714";
#endif

//частота в сети
byte Freq[]="0E0701";//{0x01,0x52,0x31,0x02,0x30,0x45,0x30,0x37,0x30,0x31,0x46,0x46,0x28,0x29,0x03,0x10};


/*
*    Сохраненные значения из счётчика 
*/
//общие показания счетчика квт*ч
//answer: all,period1,period2,period3...
byte ConsumptionKwth[] = "0F0880";//{0x01,0x52,0x31,0x02,0x30,0x46,0x30,0x38,0x38,0x30,0x46,0x46,0x28,0x29,0x03,0x15};

//температура счетчика
byte Temp[]="600900";//{0x01,0x52,0x31,0x02,0x36,0x30,0x30,0x39,0x30,0x30,0x46,0x46,0x28,0x29,0x03,0x6C};


/*
*    Данные счётчика
*/
//серийный номер счетчика
byte SerNo[]="600100";//{0x01,0x52,0x31,0x02,0x36,0x30,0x30,0x31,0x30,0x30,0x46,0x46,0x28,0x29,0x03,0x64};

//дата в счетчике
byte Date[]="000902";//{0x01,0x52,0x31,0x02,0x30,0x30,0x30,0x39,0x30,0x32,0x46,0x46,0x28,0x29,0x03,0x68};

//время в счетчике
byte Time[]={0x01,0x52,0x31,0x02,0x30,0x30,0x30,0x39,0x30,0x31,0x46,0x46,0x28,0x29,0x03,0x6B};

//напряжение встроенной батареи
byte VoltageBat[] = "600603";


/*
 * Другие переменные, связанные с запросами
 * Обычный запрос выглядит примерно так: .R1.0A0164FF()..
 * но нам это неудобно, т.к. всего 6 символов в запросах меняются,
 * а с остальными только проблемы. Поэтому добавлены возможность 
 * указать только 6 символов, а постоянные символы идущие ДО и ПОСЛЕ
 * будут добавлены автоматически. так же будет посчитана контрольная сумма.
 * 
 * Это работает и из переменных и при отправке запросов через Serial порт Арудино.
 */
//префикс строки - добавляется в начале строки с короткой записью (типа: 600603)
byte DefaultPrefix[] = {0x01,0x52,0x31,0x02};

//суфикс строки - добавляется в конце строки с короткой записью (типа: 600603)
byte DefaultSuffix[] = {0x46,0x46, 0x28,0x29,0x03};


void setup()
{
 
 pinMode(OCR1A_OUT_PIN, OUTPUT);
 
 pinMode(IS_CONNECTED_METER_LED_PIN, OUTPUT);
 digitalWrite(IS_CONNECTED_METER_LED_PIN, LOW);

//https://forum.arduino.cc/t/using-timer1-alternately-spi-and-38khz-pwm/372385
 // Clear Timer on Compare Match (CTC) Mode
 bitWrite(TCCR1A, WGM10, 0);
 bitWrite(TCCR1A, WGM11, 0);
 bitWrite(TCCR1B, WGM12, 1);
 bitWrite(TCCR1B, WGM13, 0);

 // Toggle OC1A and OC1B on Compare Match.
 bitWrite(TCCR1A, COM1A0, 1);
 bitWrite(TCCR1A, COM1A1, 0);
 bitWrite(TCCR1A, COM1B0, 1);
 bitWrite(TCCR1A, COM1B1, 0);

 // No prescaling
 bitWrite(TCCR1B, CS10, 1);
 bitWrite(TCCR1B, CS11, 0);
 bitWrite(TCCR1B, CS12, 0);

  /*
   * Для свтодиода нужно 38кгц.
   * Считаем OCR1A и OCR1B по такому принципу:
   * OCR1A = (частота_микроконтроллера_в_гц / 38000кгц) / 2;
   * OCR1B = OCR1A
   * 
   * Например, частота МК 16мГц, тогда:
   * 16000000/38000 = 421.05
   * 421/2 = 210
   * Значит OCR1A и OCR1B указываем 210
   */
 OCR1A = 210;
 OCR1B = 210;



  
  // Открываем стандартный порт:
  Serial.begin(SERIAL_SPEED);//SERIAL_7E1
  while (!Serial) {
    ; // wait for serial port to connect. Needed for Leonardo only
  }
  
  Serial.println("Start!");
  Serial.print("Wait:");
  Serial.print(PAUSE_AFTER_START);
  Serial.println("ms...");
  
  delay(PAUSE_AFTER_START);

  //инициализация счетчика и вывод даты времени
  InitMeter(true);   
}

void loop()
{ 
  //настало время чтения параметров электросети
  if(COUNTER_electricity_data <= 0)
  {
    COUNTER_electricity_data = NEXT_ELECTRICITY_DATA_REQUEST_IN;
    
    ReadParam("Freq", Freq, sizeof(Freq));

    //3-фазный счетчик
    #ifdef THREE_PHASE_METER
      ReadParam("VoltageP1", VoltageP1, sizeof(VoltageP1));
      ReadParam("VoltageP2", VoltageP2, sizeof(VoltageP2));
      ReadParam("VoltageP3", VoltageP3, sizeof(VoltageP3));

      ReadParam("CurrentP1", CurrentP1, sizeof(CurrentP1));
      ReadParam("CurrentP2", CurrentP2, sizeof(CurrentP2));
      ReadParam("CurrentP3", CurrentP3, sizeof(CurrentP3));

      ReadParam("APowerP1", APowerP1, sizeof(APowerP1));
      ReadParam("APowerP2", APowerP2, sizeof(APowerP2));
      ReadParam("APowerP3", APowerP3, sizeof(APowerP3));
      ReadParam("APowerSum", APowerSum, sizeof(APowerSum));

      ReadParam("PowerFactorP1", PowerFactorP1, sizeof(PowerFactorP1));
      ReadParam("PowerFactorP2", PowerFactorP2, sizeof(PowerFactorP2));
      ReadParam("PowerFactorP3", PowerFactorP3, sizeof(PowerFactorP3));
      ReadParam("PowerFactorSum", PowerFactorSum, sizeof(PowerFactorSum));

      if(EXTENDED_DATA_NEEDED)
      {
        ReadParam("RPowerNegativeP1", RPowerNegativeP1, sizeof(RPowerNegativeP1));
        ReadParam("RPowerNegativeP2", RPowerNegativeP2, sizeof(RPowerNegativeP2));
        ReadParam("RPowerNegativeP3", RPowerNegativeP3, sizeof(RPowerNegativeP3));
        ReadParam("RPowerNegativeSum", RPowerNegativeSum, sizeof(RPowerNegativeSum));

        ReadParam("RPowerPositiveP1", RPowerPositiveP1, sizeof(RPowerPositiveP1));
        ReadParam("RPowerPositiveP2", RPowerPositiveP2, sizeof(RPowerPositiveP2));
        ReadParam("RPowerPositiveP3", RPowerPositiveP3, sizeof(RPowerPositiveP3));
        ReadParam("RPowerPositiveSum", RPowerPositiveSum, sizeof(RPowerPositiveSum));  

        ReadParam("AngleVoltageP1_P2", AngleVoltageP1_P2, sizeof(AngleVoltageP1_P2));
        ReadParam("AngleVoltageP2_P3", AngleVoltageP2_P3, sizeof(AngleVoltageP2_P3));
        ReadParam("AngleVoltageP1_P3", AngleVoltageP1_P3, sizeof(AngleVoltageP1_P3));                       
      }

      //это был долгий запрос, поэтому скорректируем счетчики запросов остальных данных
      COUNTER_meter_values = COUNTER_meter_values - 3;
      COUNTER_meter_info = COUNTER_meter_info - 3;

    //1-фазный счетчик
    #else
      ReadParam("Voltage", Voltage, sizeof(Voltage));

      ReadParam("Current", Current, sizeof(Current));

      ReadParam("APower", APower, sizeof(APower));

      ReadParam("PowerFactor", PowerFactor, sizeof(PowerFactor)); 

      if(EXTENDED_DATA_NEEDED)
      {
        ReadParam("RPowerPositive", RPowerPositive, sizeof(RPowerPositive));
        ReadParam("RPowerNegative", RPowerNegative, sizeof(RPowerNegative));      
      }
      
      //это был долгий запрос, поэтому скорректируем счетчики запросов остальных данных
      COUNTER_meter_values = COUNTER_meter_values - 1;
      COUNTER_meter_info = COUNTER_meter_info - 1;
    #endif


  
    //настало время получить данные из памяти счетчика (расход эл-энергии, температуру)
    if(COUNTER_meter_values <= 0 && NEXT_METER_VALUES_REQUEST_IN)
    {
      COUNTER_meter_values = NEXT_METER_VALUES_REQUEST_IN;
      ReadParam("ConsumptionKwth", ConsumptionKwth, sizeof(ConsumptionKwth));
      ReadParam("Temp", Temp, sizeof(Temp));
    }

    //настало время проконтролировать данные самого счетчика
    if(COUNTER_meter_info <= 0 && NEXT_METER_REQUEST_IN)
    {
      COUNTER_meter_info = NEXT_METER_REQUEST_IN; 
  
      ReadParam("SerNo", SerNo, sizeof(SerNo));
      ReadParam("Date", Date, sizeof(Date));
      ReadParam("Time", Time, sizeof(Time));
      ReadParam("VoltageBat", VoltageBat, sizeof(VoltageBat)); 
    }
    
    Serial.print("\n");
    COUNTER_ping++;
  }
  else{
    COUNTER_ping--;
  }

  //пингуем счетчик, чтобы каждый раз не инициализировать
  if(NEXT_PING_IN && COUNTER_ping == 0 && isConnected)
  {
    COUNTER_ping =  NEXT_PING_IN; 
    writeShortString(SerNo, sizeof(SerNo));
    readDataFromMeter();
    //Serial.print("ping:");
    //Serial.println(readDataFromMeter());
  }
  
  COUNTER_electricity_data--;
  COUNTER_meter_values--;
  COUNTER_meter_info--;


  /*
   * Прием данных из Serial-порта (например с компа) и отправка обратно ответа из счетчика
   */
  #ifdef WAIT_REQUESTS_FROM_UART
    inputStringFromSerial = "";
    
    while (Serial.available()) 
    {
      inChar = Serial.read();
      if (inChar == '\r' || inChar == '\n')continue;
      inputStringFromSerial += inChar;
    }

    if( inputStringFromSerial != "")
    {
      Serial.print("RQST:[");
      Serial.print(inputStringFromSerial);

      //короткая команда пришла
      if(inputStringFromSerial[0] != 0x01)
      {
        //возьмем первые несколько байт (SHORT_REQUEST_LENGTH)
        for(iter=0; iter<SHORT_REQUEST_LENGTH; iter++)
        {
          inputShort[iter] = inputStringFromSerial[iter]; 
        }
        writeShortString(inputShort, SHORT_REQUEST_LENGTH);  
      }
      //если строка начиналась с 0x01, значит был запрос по полной строке запроса 
      //целиком ей отправим в счетчик
      else
      {
        //send request
        MeterSerial.print( inputStringFromSerial );
      }

      //читаем ответ
      Serial.print("]ANSWR:{");
      Serial.print( readDataFromMeter() );
      Serial.println("}");
      
    }
  #endif


  delay(1000);

  //мигаем светодиодом
  if( isConnected )
  {
    digitalWrite(IS_CONNECTED_METER_LED_PIN, ! digitalRead(IS_CONNECTED_METER_LED_PIN) );
  }
  else
  {
    digitalWrite(IS_CONNECTED_METER_LED_PIN, HIGH );
    delay(50);
    digitalWrite(IS_CONNECTED_METER_LED_PIN, LOW );

    //если счетчик не отвечаем, пробуем начать обмен заново
    InitMeter(0);
  }

}











/*
 * Инициализация счетчика.
 * GetDateTime - получать ли дату и время после успешного подключения
 * 
 * 1. Отправка строки инициализации /!?
 * 2. Получение ответа
 * 3. Смена скорости.
 * 4. Отправка пароля
 * 5. Получение даты и вемени (если запрошено)
 */
void InitMeter( bool GetDateTime )
{
  //off led
  digitalWrite(IS_CONNECTED_METER_LED_PIN, LOW);

  Serial.println("Init meter");

  //change speed
  if( MeterSerial.isListening() )
  {
    MeterSerial.end();  
  }
  MeterSerial.begin(INIT_SPEED);
  MeterSerial.listen();

  int WaitMeter = 0;

  //repeat connect
  while( ! MeterSerial.available() && WaitMeter < NUMBER_OF_ATTEMPTS_TO_CONNECT)
  {
    if( WaitMeter )
    {
      Serial.println("Noanswer");
      delay(2000);
    }

    //send init command
    MeterSerial.write(Init, sizeof(Init));
    delay(150);
    
    WaitMeter++;
  }  

  if(WaitMeter < NUMBER_OF_ATTEMPTS_TO_CONNECT && MeterSerial.available() )
  {
    Serial.print("Connect to meter: ");
    Serial.println( readDataFromMeter() );
    
    digitalWrite(IS_CONNECTED_METER_LED_PIN, HIGH);
    isConnected=true;
  }
  else
  {
    Serial.println("NotConnect");
    return;
  }

  //send confirm
  MeterSerial.write(OkAndChangeSpeed, sizeof(OkAndChangeSpeed)); 
    delay(500);


  //change speed
  delay(600);
  MeterSerial.end();
  MeterSerial.begin(BASE_SPEED);
  MeterSerial.listen();  

  //send pass
  MeterSerial.write(Pass, sizeof(Pass));
  // Drop the rest of the pending frames. No time to draw them now!
  while (MeterSerial.available()) {
    MeterSerial.read();
  }  


  delay(700);
  //get date & time
  if( GetDateTime )
  {
    ReadParam("Date", Date, sizeof(Date));
    ReadParam("Time", Time, sizeof(Time));  
    Serial.println(outputString);
  }
}

/*
 * Читаем данные из счетчика,
 * удаляем весь непечатный мусор
 * и возвращаем данные
 */
String readDataFromMeter()
{
  isConnected=0;
  bool skipNext=false;
  String _read = "";
  
  delay(150);
  while (MeterSerial.available() > 0) 
  {
    isConnected=1;  
    
    incomingByte = MeterSerial.read();
   
    incomingByte &= ~(1 << 7);    // forces 0th bit of x to be 0.  all other bits left alone.

    //end of data
    if( incomingByte == 0x03)
    {
      skipNext=true;
    }

    //skip non printable symbols
    if( incomingByte < 0x21 || incomingByte > 0xC0 || skipNext)
    {
      continue;
    }
    
    _read += (char)incomingByte;
  }

  return _read;
}

/*
 * Отправка запросов из короткой строки вида 0102F5
 * data - массив с запросом
 * sz - длина массива с запросом
 */
void writeShortString(byte data[], int sz)
{
      byte _data[16];
      char bccData;
      
      memcpy(_data, DefaultPrefix, sizeof(DefaultPrefix));
      memcpy(_data + sizeof(DefaultPrefix), data, SHORT_REQUEST_LENGTH);
      memcpy(_data + sizeof(DefaultPrefix)+SHORT_REQUEST_LENGTH, DefaultSuffix, sizeof(DefaultSuffix));
      bccData =  bcc(_data);

      memcpy(_data + sizeof(DefaultPrefix) + SHORT_REQUEST_LENGTH + sizeof(DefaultSuffix), &bccData, 1);
      MeterSerial.write(_data, 16); 
}

/*
 * Чтение параметра и вывод ответа в Serial
 * param - название запроса
 * data - массив с запросом
 * sz - длина массива с запросом
 */
void ReadParam(String param, byte data[], int sz)
{
  //если счётчик не подключен, то и слать запросы смысла нет
  if( ! isConnected )
  {
    return;
  }

  //выводить ли название запроса
  if( param != "" && ! OUTPUT_WITHOUT_NAME)
  {
    outputString += param+":";
  }

  if(sz > 1  )
  {
    if(sz == (SHORT_REQUEST_LENGTH+1) || sz == SHORT_REQUEST_LENGTH)
    {
      writeShortString(data, sz);
    }
    else
    {
    MeterSerial.write(data, 16);
    }
    outputString += readDataFromMeter();
  }
  
  if( param != "")
  {
    outputString += "|";
  }
  Serial.print(outputString);
  outputString="";
  delay(PAUSE_AFTER_QUERY);
}


/*
 * Вычисление XOR для запроса
 *  data - массив с запросом
 */
char bcc(char *data)
{
  char bcc=0; 
  for(int i=1; i<15; i++)
  {
   bcc ^=data[i];
  }
  return bcc;
}
