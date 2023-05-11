const dirIcon = Vue.prototype.$global.board.board_info.dir;
module.exports = {
  blocks: [ // use "blocks : [ " in normally situation but this need to override base block from esp-idf platforms
    {
      name: "LED Matrix",
      color: "230",
      //icon: "/static/icons/icons8_picture_96px_1.png",
      icon: `file:///${dirIcon}/static/icons/ledmatrix.png`,
      blocks: [
        {
          xml: `<sep gap="32"></sep><label text="LCD TFT Select Mode" web-class="headline"></label>`
        },
        "tft_display_selectMode0",
        "tft_display_setcolorText",
        "tft_display_setcolorBg",
        {
          xml: `<sep gap="32"></sep><label text="LED Matrix 16x8" web-class="headline"></label>`
        },
        "display_led16x8",
        "display_led16x8_clr",
        {
          xml: `<block type="display_led16x8_print">
                <value name="VALUE">
                    <shadow type="basic_string">
                      <field name="VALUE">Hello world!</field>
                    </shadow>
                </value>
              </block>` },
        {
          xml: `<block type="display_led16x8_scroll">
                <value name="VALUE">
                    <shadow type="basic_string">
                      <field name="VALUE">Hello world!</field>
                    </shadow>
                </value>
              </block>`
        },
        'basic_string',
        {
          xml: `<sep gap="32"></sep><label text="Advanced 16x8 Display" web-class="headline"></label>`
        },
        {
          xml: `<block type="display_led16x8_drawPixel">
                <value name="X">
                    <shadow type="math_number">
                      <field name="NUM">0</field>
                    </shadow>
                </value>
                <value name="Y">
                    <shadow type="math_number">
                      <field name="NUM">0</field>
                    </shadow>
                </value>
                </block>`
        },
        {
          xml: `<block type="display_led16x8_drawline">
                <value name="X0">
                    <shadow type="math_number">
                      <field name="NUM">0</field>
                    </shadow>
                </value>
                <value name="Y0">
                    <shadow type="math_number">
                      <field name="NUM">0</field>
                    </shadow>
                </value>
                <value name="X1">
                    <shadow type="math_number">
                      <field name="NUM">15</field>
                    </shadow>
                </value>
                <value name="Y1">
                    <shadow type="math_number">
                      <field name="NUM">7</field>
                    </shadow>
                </value>
                </block>`
        },
        {
          xml: `<block type="display_led16x8_drawRect">
                <value name="X">
                    <shadow type="math_number">
                      <field name="NUM">5</field>
                    </shadow>
                </value>
                <value name="Y">
                    <shadow type="math_number">
                      <field name="NUM">1</field>
                    </shadow>
                </value>
                <value name="W">
                    <shadow type="math_number">
                      <field name="NUM">6</field>
                    </shadow>
                </value>
                <value name="H">
                    <shadow type="math_number">
                      <field name="NUM">6</field>
                    </shadow>
                </value>
                </block>`
        },
        {
          xml: `<block type="display_led16x8_drawcircle">
                <value name="X">
                    <shadow type="math_number">
                      <field name="NUM">7</field>
                    </shadow>
                </value>
                <value name="Y">
                    <shadow type="math_number">
                      <field name="NUM">3</field>
                    </shadow>
                </value>
                <value name="R">
                    <shadow type="math_number">
                      <field name="NUM">2</field>
                    </shadow>
                </value>
                </block>`
        },
        {
          xml: `<block type="display_led16x8_drawTriangle">
                <value name="X0">
                    <shadow type="math_number">
                      <field name="NUM">3</field>
                    </shadow>
                </value>
                <value name="Y0">
                    <shadow type="math_number">
                      <field name="NUM">6</field>
                    </shadow>
                </value>
                <value name="X1">
                    <shadow type="math_number">
                      <field name="NUM">13</field>
                    </shadow>
                </value>
                <value name="Y1">
                    <shadow type="math_number">
                      <field name="NUM">6</field>
                    </shadow>
                </value>
                <value name="X2">
                    <shadow type="math_number">
                      <field name="NUM">8</field>
                    </shadow>
                </value>
                <value name="Y2">
                    <shadow type="math_number">
                      <field name="NUM">1</field>
                    </shadow>
                </value>
                </block>`
        }
        // 'basic_TFT_setRotation',
        // 'basic_TFT_fillScreen',
        // 'basic_TFT_setTextSize',
        // 'basic_TFT_print'
      ]
    },
    {
      name: "LCD Display",
      color: "230",
      icon: `file:///${dirIcon}/static/icons/graphictablet.png`,
      blocks: [
        {
          xml: `<sep gap="32"></sep><label text="LCD TFT Select Mode" web-class="headline"></label>`
        },
        "tft_display_selectMode1",
        {
          xml: `<sep gap="32"></sep><label text="LCD TFT 0.96 inch 160x80 Pixels" web-class="headline"></label>`
        },
        {
          xml: `<block type="variables_set">
                                     <field name="VAR">img1</field>
                                     <value name="VALUE">
                                         <block type="i2c128x64_create_image" inline="false"></block>
                                     </value>
                                 </block>`
        },
        {
          xml:
            `<block type="i2c128x64_display_image">
                             <value name="img">
                                 <block type="variables_get">
                                     <field name="VAR">img1</field>
                                 </block>
                             </value>
                             <value name="x">
                                 <shadow type="math_number">
                                     <field name="NUM">0</field>
                                 </shadow>
                             </value>
                             <value name="x">
                                 <shadow type="math_number">
                                     <field name="NUM">0</field>
                                 </shadow>
                             </value>
                             <value name="y">
                                 <shadow type="math_number">
                                     <field name="NUM">0</field>
                                 </shadow>
                             </value>
                             <value name="width">
                                 <shadow type="math_number">
                                     <field name="NUM">10</field>
                                 </shadow>
                             </value>
                             <value name="height">
                                 <shadow type="math_number">
                                     <field name="NUM">10</field>
                                 </shadow>
                             </value>
                         </block>`
        },
        "tft_display_setRotation",
        "tft_display_fillScreen",
        //'basic_TFT_setFonts',
        {
          xml:
            `<block type="tft_display_print">
                            <value name="TEXT">
                                <shadow type="basic_string">
                                    <field name="VALUE">Hello world!</field>
                                </shadow>
                            </value>
                            <value name="X">
                                <shadow type="math_number">
                                    <field name="NUM">0</field>
                                </shadow>
                            </value>
                            <value name="Y">
                                <shadow type="math_number">
                                    <field name="NUM">0</field>
                                </shadow>
                            </value>
                        </block>`
        },
        {
          xml:
            `<block type='basic_TFT_print_TH'>
                            <value name="TEXT">
                                <shadow type="basic_string">
                                    <field name="VALUE">Hello world!</field>
                                </shadow>
                            </value>
                            <value name="X">
                                <shadow type="math_number">
                                    <field name="NUM">0</field>
                                </shadow>
                            </value>
                            <value name="Y">
                                <shadow type="math_number">
                                    <field name="NUM">0</field>
                                </shadow>
                            </value>
                        </block>`
        },
        {
          xml: `<sep gap="32"></sep><label text="Shape" web-class="headline"></label>`
        },
        {
          xml:
            `<block type="tft_display_draw_line">
                            <value name="x0">
                                <shadow type="math_number">
                                    <field name="NUM">10</field>
                                </shadow>
                            </value>
                            <value name="y0">
                                <shadow type="math_number">
                                    <field name="NUM">10</field>
                                </shadow>
                            </value>
                            <value name="x1">
                                <shadow type="math_number">
                                    <field name="NUM">100</field>
                                </shadow>
                            </value>
                            <value name="y1">
                                <shadow type="math_number">
                                    <field name="NUM">50</field>
                                </shadow>
                            </value>
                        </block>`
        },
        {
          xml:
            `<block type="tft_display_draw_rect">
                            <value name="x">
                                <shadow type="math_number">
                                    <field name="NUM">10</field>
                                </shadow>
                            </value>
                            <value name="y">
                                <shadow type="math_number">
                                    <field name="NUM">10</field>
                                </shadow>
                            </value>
                            <value name="width">
                                <shadow type="math_number">
                                    <field name="NUM">50</field>
                                </shadow>
                            </value>
                            <value name="height">
                                <shadow type="math_number">
                                    <field name="NUM">30</field>
                                </shadow>
                            </value>
                        </block>`
        },
        {
          xml:
            `<block type="tft_display_draw_circle">
                            <value name="x">
                                <shadow type="math_number">
                                    <field name="NUM">64</field>
                                </shadow>
                            </value>
                            <value name="y">
                                <shadow type="math_number">
                                    <field name="NUM">32</field>
                                </shadow>
                            </value>
                            <value name="r">
                                <shadow type="math_number">
                                    <field name="NUM">20</field>
                                </shadow>
                            </value>
                        </block>`
        },
        {
          xml:
            `<block type="i2c128x64_display_draw_pixel">
                              <value name="x">
                                  <shadow type="math_number">
                                      <field name="NUM">0</field>
                                  </shadow>
                              </value>
                              <value name="y">
                                  <shadow type="math_number">
                                      <field name="NUM">0</field>
                                  </shadow>
                              </value>
                          </block>`
        },
        "basic_string"
      ]
    },

    {
      name: "Remote XY",
      color: "58",
      icon: `file:///${dirIcon}/static/icons/REMOTE.png`,
      blocks: [
        {
          xml: `<sep gap="32"></sep><label text="Setup Remote" web-class="headline"></label>`
        },
        "remote_xy_begin",
        {
          xml: `<sep gap="32"></sep><label text="Read Data" web-class="headline"></label>`
        },
        "remote_xy_get",
      ]
    },

    {
      name: "Drone Kit",
      color: "58",
      icon: `file:///${dirIcon}/static/icons/MPU6050.png`,
      blocks: [
        {
          xml: `<sep gap="32"></sep><label text="Setup Drone" web-class="headline"></label>`
        },
        "remotemabegin",
        "remotexybegin",
        "dronebegin",
        "dronecal",
        {
          xml: `<sep gap="32"></sep><label text="Run Drone" web-class="headline"></label>`
        },
        "remotemarun",
        "remotexyrun",
        //"dronerun",
        {
          xml: `<sep gap="32"></sep><label text="Batterry" web-class="headline"></label>`
        },
        "drone_readbat",
        // {
        //   xml: `<sep gap="32"></sep><label text="Temperature (°C)" web-class="headline"></label>`
        // },
        // "sensor_tempmpu6050",
        {
          xml: `<sep gap="32"></sep><label text="Time-of-flight" web-class="headline"></label>`
        },
        "sensor_tof",
      ]
    },

    {
      name: "Sensor",
      color: "58",
      icon: "/static/icons/icons8_thermometer_96px.png",
      blocks: [
        {
          xml: `<sep gap="32"></sep><label text="Kidbright32 Sensor" web-class="headline"></label>`
        },
        "sensor_ldr",
        "sensor_lm73",
        "sensor_switch1",
        "sensor_switch2",
        {
          xml: `<sep gap="32"></sep><label text="Ambient Light Sensor" web-class="headline"></label>`
        },
        "sensor_lux",
        "sensor_luxhighgain",
        "sensor_luxlowgain",
        {
          xml: `<sep gap="32"></sep><label text="MPU6050" web-class="headline"></label>`
        },
        "sensor_setmpu6050",
        "sensor_accmpu6050",
        "sensor_gyrompu6050",
        "sensor_kalAngle",
        // {
        //   xml: `<sep gap="32"></sep><label text="Gyroscope (°/s)" web-class="headline"></label>`
        // },
        // "sensor_gyrompu6050",
        // // {
        // //   xml: `<sep gap="32"></sep><label text="Temperature (°C)" web-class="headline"></label>`
        // // },
        // // "sensor_tempmpu6050",
        // {
        //   xml: `<sep gap="32"></sep><label text="Angle of Rotation (°)" web-class="headline"></label>`
        // },
        // "sensor_kalAngle",

      ]
    },
    {
      name: "Servo",
      color: "58",
      icon: `file:///${dirIcon}/static/icons/servo.png`,
      blocks: [
        "esp32_servo_attach",
        "esp32_servo_detach",
        "esp32_servo_write",
        "esp32_servo_write_micros",
        "esp32_servo_read",
        "esp32_servo_read_micros",
        {
          xml: `<sep gap="32"></sep><label text="Gripper" web-class="headline"></label>`
        },
        
        "esp32_servo_gripper_setup",

        {
          xml:
            `<block type="esp32_servo_gripper_up">
    <value name="SPEED">
        <shadow type="math_number">
            <field name="NUM">50</field>
        </shadow>
    </value>
</block>`
        },

        {
          xml:
            `<block type="esp32_servo_gripper_down">
  <value name="SPEED">
      <shadow type="math_number">
          <field name="NUM">50</field>
      </shadow>
  </value>
</block>`
        },

        {
          xml:
            `<block type="esp32_servo_gripper_release">
<value name="SPEED">
    <shadow type="math_number">
        <field name="NUM">50</field>
    </shadow>
</value>
</block>`
        },

        {
          xml:
            `<block type="esp32_servo_gripper_arrest">
<value name="degree">
    <shadow type="math_number">
        <field name="NUM">70</field>
    </shadow>
</value>
<value name="SPEED">
    <shadow type="math_number">
        <field name="NUM">50</field>
    </shadow>
</value>
</block>`
        },


      ]
    },
    // {
    //   name: "MQTT",
    //   color: "230",
    //   icon: `file:///${dirIcon}/static/icons/mqtt.png`,
    //   blocks: [
    //      "mqtt_connector_begin",
    //      "on_prepare_data",
    //      "on_message",
    //      "append_value",
    //      "mqtt_connect"
    //   ]
    // },
    {
      name: "GPIO",
      color: "30",
      icon: "/static/icons/icons8_electronics_96px.png",
      blocks: [
        {
          xml: `<sep gap="32"></sep><label text="KidBright GPIO" web-class="headline"></label>`
        },
        "output_write",
        "output_toggle",
        "output_read",
        "usbsw_write",
        "usbsw_toggle",
        "usbsw_read",
        "input_read"
      ]
    },
    {
      name: "Music",
      color: "330",
      icon: "/static/icons/SVG/c6.svg",
      blocks: [
        "music_note",
        "music_notes",
        {
          xml:
            `<block type="music_play_notes">
                        <value name="note">                    
                            <block type="music_notes">
                                <field name="notes">C4,B4,E4</field>
                            </block>
                        </value>
                    </block>`
        },
        'music_song_mario_underworld',
        'music_song_jingle_bell',
        'music_song_cannon_rock'
        // 'music_rest',
        // 'music_scale',
        // 'music_set_volume',
        // 'music_get_volume'
      ]
    },
    {
      name: "Time",
      color: "330",
      icon: "/static/icons/SVG/c6.svg",
      blocks: [
        {
          xml:
            `<block type="time_delay">
                        <value name="delay">
                            <shadow type="math_number">
                                <field name="NUM">1000</field>
                            </shadow>
                        </value>
                    </block>`
        },

        {
          xml:
            `<block type="time_delay_microsec">
                        <value name="delay">
                            <shadow type="math_number">
                                <field name="NUM">1000</field>
                            </shadow>
                        </value>
                    </block>`
        },
        {
          xml: `<sep gap="32"></sep><label text="Real Time Clock" web-class="headline"></label>`
        },
        "mcp7941_rtc_set_datetime",
        "mcp7941_rtc_get_dayOfWeek",
        "mcp7941_rtc_get_hour",
        "mcp7941_rtc_get_minute",
        "mcp7941_rtc_get_second",
        "mcp7941_rtc_get_day",
        "mcp7941_rtc_get_month",
        "mcp7941_rtc_get_year"
      ]
    }
  ]
};
