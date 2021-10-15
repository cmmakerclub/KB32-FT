const dirIcon = Vue.prototype.$global.board.board_info.dir;
module.exports = function (Blockly) {
  "use strict";

  let remotexy_username = "";
  let remotexy_password = "";

  Blockly.JavaScript["remote_xy_begin"] = function (block) {
    var text_remotexy_username = block.getFieldValue("USERNAME");
    var text_remotexy_password = block.getFieldValue("PASSWORD");
    var text_remotexy_port = block.getFieldValue("PORT");
    //var text_mqtt_client_id = block.getFieldValue("MQTT_CLIENT_ID");

    remotexy_username = text_remotexy_username;
    remotexy_password = text_remotexy_password;

    // TODO: Assemble JavaScript into code variable.
    var code = `
    #EXTINC #define REMOTEXY_MODE__ESP32CORE_WIFI_POINT #END
    #EXTINC #include <RemoteXY.h> #END
    #EXTINC #define REMOTEXY_WIFI_SSID "${text_remotexy_username}" #END
    #EXTINC #define REMOTEXY_WIFI_PASSWORD "${text_remotexy_password}" #END
    #EXTINC #define REMOTEXY_SERVER_PORT ${text_remotexy_port} #END

    #FUNCTION
    // RemoteXY configurate
    #pragma pack(push, 1)
    uint8_t RemoteXY_CONF[] = {255, 5,   0,  1,  0,   70,  0,   11,  13,  0,   5,
                               32,  0,   8,  30, 30,  1,   26,  31,  5,   32,  70,
                               8,   30,  30, 1,  26,  31,  2,   0,   39,  2,   22,
                               11,  36,  26, 31, 31,  79,  78,  0,   79,  70,  70,
                               0,   66,  0,  46, 30,  9,   29,  37,  26,  129, 0,
                               39,  14,  22, 3,  24,  65,  108, 116, 105, 116, 117,
                               100, 101, 32, 67, 111, 110, 116, 114, 111, 108, 0};

    // this structure defines all the variables and events of your control interface 
    struct {

      // input variables
      int8_t joystick_1_x; // =-100..100 x-coordinate joystick position 
      int8_t joystick_1_y; // =-100..100 y-coordinate joystick position 
      int8_t joystick_2_x; // =-100..100 x-coordinate joystick position 
      int8_t joystick_2_y; // =-100..100 y-coordinate joystick position 
      uint8_t switch_1; // =1 if switch ON and =0 if OFF 

      // output variables
      int8_t level_1; // =0..100 level position 

        // other variable
      uint8_t connect_flag;  // =1 if wire connected, else =0 

    } RemoteXY;

    #pragma pack(pop)

    float remote_xy_Read (uint8_t index){
      float tmp = -1;

      switch(index){
        case 1:
          tmp = RemoteXY.joystick_1_x;
          break;
        case 2:
          tmp = RemoteXY.joystick_1_y;
          break;
        case 3:
          tmp = RemoteXY.joystick_2_x;
          break;
        case 4:
          tmp = RemoteXY.joystick_2_y;
          break;
        case 5:
          tmp = RemoteXY.switch_1;
          break;
      }

      return tmp;
    }

    void xy_handle(void* pvParameters)  // This is a task.
    {
      (void)pvParameters;
      uint32_t start_time = xTaskGetTickCount();
      for (;;)
      {
        RemoteXY_Handler();
        vTaskDelayUntil(&start_time, 30);
      }
    }
    /////////////////////////////////////////////
    //           END RemoteXY include          //
    /////////////////////////////////////////////

    #END

    #SETUP
    // RemoteXY connection settings

    RemoteXY_Init();
    delay(100);

    xTaskCreatePinnedToCore(
      xy_handle
      , "xy_handle"
      , 1024 * 2
      , NULL
      , 1
      , NULL
      , 1);

    #END
    \n
    `;
    return code;
  };



  Blockly.JavaScript['remote_xy_get'] = function (block) {
		return [
			'remote_xy_Read(' + block.getFieldValue('OUTPUT') + ')',
			Blockly.JavaScript.ORDER_ATOMIC
		];
	};

};