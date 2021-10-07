/*
  SimpleKNN

  This example demonstrates how to use the Arduino KNN library.

  It creates a KNN classifier that expects an input array of 2 (floats).
  Then adds 4 example inputs with their respective classifications.
  After this it demonstrates how to classify an input value and get
  the classifications confidence.

  This example code is in the public domain.
*/
// Include Arduino FreeRTOS library
#include <Arduino_FreeRTOS.h>
#include <Arduino_LSM9DS1.h>
#include <Arduino_KNN.h>

// Create a new KNNClassifier, input values are array of 2 (floats),
// change if you need a bigger input size
  float aX, aY, aZ, gX, gY, gZ;

KNNClassifier myKNN(6);


TaskHandle_t taskBlinkHandle;

TaskHandle_t taskBlinkTwiceHandle;


TaskHandle_t taskDeletedHandle;

TaskHandle_t taskBlockedHandle;

TaskHandle_t taskIMUHandle;

TaskHandle_t taskKNNClassifyHandle;

void setup() {
  Serial.begin(9600);
  while (!Serial);

  Serial.println("Simple KNN");
  Serial.println();

  Serial.print("Adding examples to myKNN ... ");
  Serial.println();

  // add examples to KNN
  float example1[] = { 7.0, 7.0,7.0,7.0,7.0,7.0 };
  float example2[] = { 5.0, 5.0 ,5.0,5.0,5.0,5.0};
  float example3[] = { 9.0, 9.0 ,9.0,9.0,9.0,9.0};
  float example4[] = { 5.0, 5.0 ,5.0,5.0,5.0,5.0};

  myKNN.addExample(example1, 7); // add example for class 7
  myKNN.addExample(example2, 5); // add example for class 5
  myKNN.addExample(example3, 9); // add example for class 9
  myKNN.addExample(example4, 5); // add example for class 5 (again)

  // get and print out the KNN count
  Serial.print("\tmyKNN.getCount() = ");
  Serial.println(myKNN.getCount());
  Serial.println();

  // you can also print the counts by class
  //  Serial.print("\tmyKNN.getCountByClass(5) = ");
  //  Serial.println(myKNN.getCountByClass(5)); // expect 2

  // classify the input
  Serial.println("Classifying input ...");

  float input[] = { 5.0, 5.0 ,4.9,4.8,5.0,5.0};



  float punchag1[]={-0.525, -0.968,  1.432, 17.517,  76.050,  -25.391};
 float punchag2[]= {-0.614 , -1.170 , 1.336 ,19.226 , 93.201,  -21.790};
 float punchag3[]={-0.694  ,-1.350 , 1.238 ,24.170 , 96.680  ,-11.841}; 
 float punchag4[]={-0.745  ,-1.452,  1.140, 29.114,  90.637 , 4.639};
 float punchag5[]={-0.764 , -1.587 , 1.044 ,31.372 , 82.947  ,26.367};
 float punchag6[]={-0.719  ,-1.646 , 0.953, 27.954,  77.209,  45.227};
 float punchag7[]={-0.677  ,-1.638  ,0.846 ,19.104  ,71.899  ,62.622};



  float flexag1[]={-0.525, -0.968,  1.432, 17.517,  76.050,  -25.391};
 float flexag2[]= {-0.614 , -1.170 , 1.336 ,19.226 , 93.201,  -21.790};
 float flexag3[]={-0.694  ,-1.350 , 1.238 ,24.170 , 96.680  ,-11.841}; 
 float flexag4[]={-0.745  ,-1.452,  1.140, 29.114,  90.637 , 4.639};
 float flexg5[]={-0.764 , -1.587 , 1.044 ,31.372 , 82.947  ,26.367};
 float flexag6[]={-0.719  ,-1.646 , 0.953, 27.954,  77.209,  45.227};
 float flexag7[]={-0.677  ,-1.638  ,0.846 ,19.104  ,71.899  ,62.622};

 
  
  
  
  int classification = myKNN.classify(input, 3); // classify input with K=3
  float confidence = myKNN.confidence();

  // print the classification and confidence
  Serial.print("\tclassification = ");
  Serial.println(classification);

  // since there are 2 examples close to the input and K = 3,
  // expect the confidence to be: 2/3 = ~0.67
  Serial.print("\tconfidence     = ");
  Serial.println(confidence);


  /**
   * Task creation
   */
  xTaskCreate(TaskBlink, // Task function
              "Blink", // Task name
              128, // Stack size 
              NULL, 
              0, // Priority
              &taskBlinkHandle); // Task handler

  xTaskCreate(TaskSerial,
              "Serial",
              128,
              NULL, 
              2,
              NULL);

  xTaskCreate(TaskDeleted,
              "Deleted",
              64,
              NULL, 
              1,
              &taskDeletedHandle);

  xTaskCreate(TaskBlocked,
              "Blocked",
              64,
              NULL, 
              1,
              &taskBlockedHandle);

  
}

void loop() {
  // do nothing
}

void IMUReadings(void *pvParameters)
 {

 


    if (IMU.accelerationAvailable()) {
      // read the acceleration data
      IMU.readAcceleration(aX, aY, aZ);

      // sum up the absolutes
      float aSum = fabs(aX) + fabs(aY) + fabs(aZ);


      IMU.readGyroscope(gX, gY, gZ);
      
    }

}


void IMUinit(void *pvParameters)

 {

 Serial.begin(9600);


  // print out the samples rates of the IMUs
  Serial.print("Accelerometer sample rate = ");
  Serial.print(IMU.accelerationSampleRate());
  Serial.println(" Hz");
  Serial.print("Gyroscope sample rate = ");
  Serial.print(IMU.gyroscopeSampleRate());
  Serial.println(" Hz");

  Serial.println();

  
 }
void TaskSerial(void *pvParameters)
{
  (void) pvParameters;

  Serial.begin(9600);

  for (;;)
  {
    Serial.println("======== Tasks status ========");
    Serial.print("Tick count: ");
    Serial.print(xTaskGetTickCount());
    Serial.print(", Task count: ");
    Serial.print(uxTaskGetNumberOfTasks());

    Serial.println();
    Serial.println();

    // Serial task status
    Serial.print("- TASK ");
    Serial.print(pcTaskGetName(NULL)); // Get task name without handler https://www.freertos.org/a00021.html#pcTaskGetName
    Serial.print(", High Watermark: ");
    Serial.print(uxTaskGetStackHighWaterMark(NULL)); // https://www.freertos.org/uxTaskGetStackHighWaterMark.html 


    TaskHandle_t taskSerialHandle = xTaskGetCurrentTaskHandle(); // Get current task handle. https://www.freertos.org/a00021.html#xTaskGetCurrentTaskHandle
    
    Serial.println();

    Serial.print("- TASK ");
    Serial.print(pcTaskGetName(taskBlinkHandle)); // Get task name with handler
    Serial.print(", High Watermark: ");
    Serial.print(uxTaskGetStackHighWaterMark(taskBlinkHandle));
    Serial.println();

    Serial.print("- TASK ");
    Serial.print(pcTaskGetName(taskDeletedHandle));
    Serial.print(", High Watermark: ");
    Serial.print(uxTaskGetStackHighWaterMark(taskDeletedHandle));
    Serial.println();

    Serial.print("- TASK ");
    Serial.print(pcTaskGetName(taskBlockedHandle));
    Serial.print(", High Watermark: ");
    Serial.print(uxTaskGetStackHighWaterMark(taskBlockedHandle));
    Serial.println();
    
    Serial.println();
    
    vTaskDelay( 5000 / portTICK_PERIOD_MS );
  }
}

void TaskBlink(void *pvParameters)
{

  pinMode(LED_BUILTIN, OUTPUT);

  for (;;)
  {
    digitalWrite(LED_BUILTIN, HIGH);
    vTaskDelay( 250 / portTICK_PERIOD_MS );
    digitalWrite(LED_BUILTIN, LOW);
    vTaskDelay( 250 / portTICK_PERIOD_MS );
  }
}



void TaskBlocked(void *pvParameters) {
  (void) pvParameters;
  for (;;)
  {
    vTaskDelay( 900000 / portTICK_PERIOD_MS );  
  }
}

/**
 * Deleted tasks when run
 */
void TaskDeleted(void *pvParameters) {
  (void) pvParameters;

  vTaskDelete(NULL);
}
