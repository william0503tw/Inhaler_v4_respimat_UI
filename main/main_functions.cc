#include "main_functions.h"

#include "device_config.h"
#include "model.h"
#include "model_settings.h"
#include "audio_provider.h"
#include "feature_provider.h"
#include "pinout_config.h"
#include "buzzer.h"
#include "mpu6050.h"
#include "status.hpp"
#include "message_package.h"

#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/i2c.h"

#include <cmath>

#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/micro_log.h"
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/micro/system_setup.h"
#include "tensorflow/lite/schema/schema_generated.h"


//-------------------------- Tensorflow Lite --------------------------
const tflite::Model* model = nullptr;
tflite::MicroInterpreter* interpreter = nullptr;
TfLiteTensor* input = nullptr;
float* model_input_buffer = nullptr;

constexpr int kTensorArenaSize = 10 * 1024;
uint8_t tensor_arena[kTensorArenaSize];
//----------------------------------- --------------------------------

//---------------- General Variable -----------------
char TAG[] = "main_functions";

// tick count for summary
float valid_inhale_tick = 0 ;
float all_inhale_tick = 0 ;

// IMU 
Pose_t start_pose ;
// hanlder for imu read in background
TaskHandle_t imu_update_handler ;
// buffer for save each score in each row
static Pose_t past_pose_buffer[CONFIG_IMU_PAST_REFERENCE_BUFFER_SIZE] ;
//----------------------------------------------------

//---------- Time Stamp Variable -----------------------
long long system_start_time_stamp ;
long long inhale_start_time_stamp ;
long long click_start_time_stamp ;

//---------- Message Variable ----------------
message_package_t curr_message_package = {
    .is_stop = 0,
    .is_press = 0,
    .flowrate = 40,
    .percentage = 0,
    .sequence = UNKNOWN_STATUS
};


void setup() {

  //------------------------ Pinout Config -------------------
  gpio_set_direction(power_on_pin, GPIO_MODE_OUTPUT); // set direction output
  gpio_set_level(power_on_pin, 0);

  //--------------------- Audio Process Settings -------------------
  I2S_begin();    // start audio provider for i2s fetching sound
  feature_provider_begin();   // start feature provider for STFFT
  mpu6050_begin();

  //---------------- TensorFlow Lite Micro Settings --------------
  model = tflite::GetModel(g_model);
  if (model->version() != TFLITE_SCHEMA_VERSION) {
    MicroPrintf("Model provided is schema version %d not equal to supported "
                "version %d.", model->version(), TFLITE_SCHEMA_VERSION);
    return;
  }

  static tflite::MicroMutableOpResolver<8> micro_op_resolver;
  if (micro_op_resolver.AddConv2D() != kTfLiteOk){
    return;
  }
  if (micro_op_resolver.AddFullyConnected() != kTfLiteOk) {
    return;
  }
  if (micro_op_resolver.AddSoftmax() != kTfLiteOk) {
    return;
  }
  if (micro_op_resolver.AddReshape() != kTfLiteOk) {
    return;
  }
  if (micro_op_resolver.AddRelu() != kTfLiteOk) {
    return;
  }
  if (micro_op_resolver.AddQuantize() != kTfLiteOk) {
    return ;
  }
  if (micro_op_resolver.AddMaxPool2D() != kTfLiteOk) {
    return ;
  }
  if (micro_op_resolver.AddDequantize() != kTfLiteOk) {
    return ;
  }


  // Build an interpreter to run the model with.
  static tflite::MicroInterpreter static_interpreter(model, micro_op_resolver, tensor_arena, kTensorArenaSize);
  interpreter = &static_interpreter;

  // Allocate memory from the tensor_arena for the model's tensors.
  TfLiteStatus allocate_status = interpreter->AllocateTensors();
  if (allocate_status != kTfLiteOk) {
    MicroPrintf("AllocateTensors() failed");
    return;
  }

  // Get information about the memory area to use for the model's input.
  input = interpreter->input(0);

  // Check the inputn shape
  if ((input->dims->size != 4) || (input->dims->data[0] != 1) ||
      (input->dims->data[1] != 8 || (input->dims->data[2]) != 32 || (input->dims->data[3] != 1) || (input->type != kTfLiteFloat32))) {
    MicroPrintf("Wrong tensor parameters in model\n");
    MicroPrintf("size: %d\n", input->dims->size);
    MicroPrintf("data[0]: %d\n", input->dims->data[0]);
    MicroPrintf("data[1]: %d\n", input->dims->data[1]);
    MicroPrintf("data[2]: %d\n", input->dims->data[2]);
    MicroPrintf("data[3]: %d\n", input->dims->data[3]);
    if(input->type != kTfLiteFloat32){
      MicroPrintf("input type not same");
    }
    return;
  }
  // float
  model_input_buffer = input->data.f;
  //------------------------- End of TensorFlow Lite Micro Settings ---------------------

}

void loop() {

  switch (status){
  case SYSTEM_START:

    system_start_time_stamp = esp_timer_get_time();

    xTaskCreate((TaskFunction_t) &task_mpu6050_update_past_buffer, "updata imu", 3 * 1024, (void*) &past_pose_buffer, 5, &imu_update_handler);
    vTaskSuspend(imu_update_handler);

    play_start_sound();

    vTaskDelay(500/portTICK_PERIOD_MS);

    status = SYSTEM_RESTART ;
  break;
  //-------------------------------------------------------------
  case SYSTEM_RESTART: {
    system_start_time_stamp = esp_timer_get_time();


    curr_message_package = {
      .is_stop = 0,
      .is_press = 0,
      .flowrate = 40,
      .percentage = 0,
      .sequence = UNKNOWN_STATUS
    };
    
    play_restart_sound();

    status = SYSTEM_INHALE_CLICK_ROTATE_DETECTION ;
  }
  break;
  //-------------------------------------------------------------
  case SYSTEM_INHALE_CLICK_ROTATE_DETECTION: {
    vTaskResume(imu_update_handler);

    wait_for_go_command();

    send_start_message();

    static float frame_each_row_score[CONFIG_ROW_SIZE] = {0} ; 

    // save last two time prediction 
    uint8_t past_prediction_buffer = NOISE_INDEX ;
    

    while(1){
      long long stime = esp_timer_get_time();

      // generate one set (8x32) of data, one store in inference model input buffer, one store in frame score buffer (size -> 8)
      generate_feature_one_set(&model_input_buffer[0], &frame_each_row_score[0]);
      // find the highest score in the window
      float local_max = find_row_max(frame_each_row_score);


      // Run the model on the spectrogram input and make sure it succeeds.
      // invoke -> make predictions
      TfLiteStatus invoke_status = interpreter->Invoke();
      if (invoke_status != kTfLiteOk) {
        MicroPrintf( "Invoke failed");
        return;
      }

      // Obtain a pointer to the output tensor
      TfLiteTensor* output = interpreter->output(0);

      //Verify output tensor shape
      if((output->dims->size != 2) || (output->dims->data[0] != 1) || (output->dims->data[1] != 2) || (output->type != kTfLiteFloat32)){
        MicroPrintf("Wrong ouput dim");
        MicroPrintf("size: %d", output->dims->size);
        MicroPrintf("data[0]: %d", output->dims->data[0]);
        MicroPrintf("data[1]: %d", output->dims->data[1]);
        return ;
      }

      int max_index = 0 ;
      float max_index_value = 0 ;
      for(int i = 0 ; i < kCategoryCount ; i++){
        if(output->data.f[i] > max_index_value){
          if(output->data.f[i] > PREDICTION_IS_INHALE_THRESHOLD){
            max_index = i ;
            max_index_value = output->data.f[i] ; 
          }
        }
      }
      
      long long time_takes = esp_timer_get_time() - stime ;
      //MicroPrintf("prediction: %s: [%.2f, %.2f] | max: %.3f | %lld us", kCategoryLabels[max_index], output->data.f[0],  output->data.f[1], local_max ,time_takes);

      if(local_max > 40000){
        // intense sound happened, determine the source
        // 1. from rotation -> read past buffer that storing imu data, if it's rotation, change of rate should be large
        // 2. from click (drug actuation) -> imu data won't be lots of changes

        vTaskSuspend(imu_update_handler);

        int judgement = mpu6050_judge_is_rotate(&past_pose_buffer[0]);

        if(judgement == CLICK){
          // click before inhale

          //get data from past imu buffer that runs in background
          //if greater than threshold, means it just happened rotation, the sound was caused by rotation not click
          play_click_identified_sound();
          click_start_time_stamp = esp_timer_get_time();

          curr_message_package.is_press = YES_PRESS ;
          curr_message_package.sequence = CLICK_BEFORE_INHALE ;

          status = SYSTEM_AFTER_CLICK_DURING_1_5_SEC ;
          break ;

        }else{
          // rotation
          vTaskResume(imu_update_handler);
        }
        
      }

      if((max_index == INHALE_INDEX)){

        if(past_prediction_buffer == INHALE_INDEX){    

          // inahale first (inahale before click)

          inhale_start_time_stamp = esp_timer_get_time() - time_takes;

          play_inhale_identified_sound();

          curr_message_package.sequence = INHALE_BEFORE_CLICK ;

          status = SYSTEM_FIRST_DROP_DETECTION ;
          break ;
        }
        past_prediction_buffer = INHALE_INDEX ;
          
      }else if(max_index == NOISE_INDEX){
        past_prediction_buffer = NOISE_INDEX ;
      }

      send_message_package(curr_message_package);
    }
    }
  break;
  //----------------------------------------------------------------------------------------------------
  case SYSTEM_FIRST_DROP_DETECTION:{

    bool is_fresh_start = 1 ;
    long long temp_time_stamp_start = esp_timer_get_time() ;

    // score
    float inhale_band_score ;

    const uint8_t lower_bound_index = 10 ;
    const uint8_t upper_bound_index = 14 ;

    // avoid error on beginning
    int ignore_count = 10 ;

    // check for whether user is in stable flow
    float change_rate = 0 ;
    float last_data = 0 ;
    float curr_data = 0 ;
    long long curr_time_stamp =  esp_timer_get_time() ;
    long long last_time_stamp = esp_timer_get_time() ; // init
    float dt = 0 ;

    // buffer ignore for serial  output
    int serial_buffer_ignore_cycle_count = 0 ;

    // sums up index from 8~15
    static float temp_slice[BAND_SIZE];
    while(1){
      inhale_band_score = 0 ;

      generate_feature_one_slice(temp_slice);

      for(int i = lower_bound_index ; i <= upper_bound_index ; i++){
        inhale_band_score += temp_slice[i];
      }

      curr_data = moving_average_filter(inhale_band_score);
      curr_time_stamp = esp_timer_get_time();
      dt = static_cast<float>(curr_time_stamp - last_time_stamp) / 1000000 ;
      
      change_rate = std::abs((curr_data - last_data) / dt) ;      

      // ignore count for avoiding wrong peak of change rate due to moving average filter
      if(ignore_count != 0){
        ignore_count-- ;
      }else{

        if(is_fresh_start) {
          //printf("%lld us\n", esp_timer_get_time() - temp_time_stamp_start);
          is_fresh_start = 0 ;
        }

        //printf("WAITTING FOR FIRST DROP (dxdt, x): %.4f, %.2f\n", change_rate, curr_data);

        if((esp_timer_get_time() - inhale_start_time_stamp) > 2000000){
          // if no drop in 2 sec, failed
          //Inhale stop unexpexted! (in first drop timeout 2s)

          curr_message_package.is_stop = YES_STOP ;

          status = SYSTEM_INHALE_FAILED ;
          break;
        }

        if((curr_data < CLICK_SCORE_THRESHOLD) & (change_rate > CLICK_CHANGE_RATE_THRESHOLD)){
          // Drop Identified ! (Due to click or stop inhaling)

          curr_message_package.is_press = YES_PRESS ;

          click_start_time_stamp = esp_timer_get_time();
          status = SYSTEM_AFTER_CLICK_DURING_1_5_SEC ;
          break;
        }

        if(serial_buffer_ignore_cycle_count == 5){
          send_message_package(curr_message_package);
          serial_buffer_ignore_cycle_count = 0 ;
        }
        serial_buffer_ignore_cycle_count++ ;
      }
    
      last_time_stamp = curr_time_stamp;
      last_data = curr_data; // save current data for next usage
    }
  }
  break;
  //--------------------------------------------------------------------------------------------------------
  case SYSTEM_AFTER_CLICK_DURING_1_5_SEC: {
    // prepare data for evalution of cases
    // score
    float inhale_band_score ;
    const uint8_t lower_bound_index = 10 ;
    const uint8_t upper_bound_index = 14 ;
    // avoid error on beginning
    int ignore_count = 10 ;

    // calculate valid percetage
    valid_inhale_tick = 0 ;
    all_inhale_tick = 0 ;

    static float temp_slice[BAND_SIZE];

    // buffer ignore for serial  output
    int serial_buffer_ignore_cycle_count = 0 ;

    while(1){
      // After click (or maybe user stop inhaling), 
      long long curr_time_stamp = esp_timer_get_time() - click_start_time_stamp ;

      if(curr_time_stamp > 1500000.0){
        status = SYSTEM_AFTER_CLICK_1_5_SEC_SUMMARY ;
        break;
      }

      // get all require data for judgement     
      // ignore several data at first for stable data
      if(ignore_count != 0){

        ignore_count-- ;

      }else{

        // update band score
        inhale_band_score = 0 ;

        generate_feature_one_slice(temp_slice);

        for(int i = lower_bound_index ; i <= upper_bound_index ; i++){
          inhale_band_score += temp_slice[i];
        }
        if(inhale_band_score > 150){
          valid_inhale_tick += 1.0 ;
        }

        all_inhale_tick += 1.0 ;

        curr_message_package.percentage = (int)(100 * (valid_inhale_tick / 125)) ;
        curr_message_package.flowrate = 30 ;
      }

      if(serial_buffer_ignore_cycle_count == 5){
        send_message_package(curr_message_package);
        serial_buffer_ignore_cycle_count = 0 ;
      }
      serial_buffer_ignore_cycle_count++ ;
      
    }
  }
  break;
  //----------------------------------------------------------------------------------------------------------
  case SYSTEM_AFTER_CLICK_1_5_SEC_SUMMARY: {
    float finish_percentage = valid_inhale_tick / all_inhale_tick ;

    if(finish_percentage > 0.70){
      status = SYSTEM_INHALE_SUCCEED ;
      break ;
    }else{
      status = SYSTEM_INHALE_FAILED ;
      break ;
    }
    

    send_message_package(curr_message_package);
  }
  break;
  //----------------------------------------------------------------------------------------------------------
  case SYSTEM_INHALE_SUCCEED:{
    play_succeed_sound();

    vTaskDelay(500/portTICK_PERIOD_MS);

    curr_message_package.is_stop = YES_STOP ;

    send_message_package(curr_message_package);

    status = SYSTEM_RESTART ;
  }
  break;
  //-------------------------------------------------------------------------------------------------------------
  case SYSTEM_INHALE_FAILED:{
    play_failed_sound();

    vTaskDelay(500/portTICK_PERIOD_MS);

    curr_message_package.is_stop = YES_STOP ;

    send_message_package(curr_message_package);

    status = SYSTEM_RESTART ;
  }
  break;
  //---------------------------------------------------------------------------------------------------------------
  case SYSTEM_SHUT_DOWN: {
    gpio_set_level(power_on_pin, 0);

    vTaskDelay(500/portTICK_PERIOD_MS);

    curr_message_package.is_stop = YES_STOP ;

    send_message_package(curr_message_package);
  }
  break;
  //---------------------------------------------------------------------------------------------------------------
  }
}

