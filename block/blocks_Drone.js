module.exports = function (Blockly) {
  "use strict";

  Blockly.Blocks["remotexybegin"] = {
    init: function () {

      this.appendDummyInput()
        .appendField(new Blockly.FieldImage(
          "https://freepngimg.com/thumb/joystick/25115-4-joystick-file.png",
          24,
          24,
          "*"))
        .appendField("RemoteXY : Setup");
      this.appendDummyInput()
        .setAlign(Blockly.ALIGN_LEFT)
        .appendField("Wifi : AP Mode")
      this.appendDummyInput()
        .setAlign(Blockly.ALIGN_LEFT)
        .appendField("USERNAME")
        .appendField(new Blockly.FieldTextInput("Drone"), "USERNAME");
      this.appendDummyInput()
        .setAlign(Blockly.ALIGN_LEFT)
        .appendField("PASSWORD")
        .appendField(new Blockly.FieldTextInput("12345678"), "PASSWORD");
      this.appendDummyInput()
        .setAlign(Blockly.ALIGN_LEFT)
        .appendField("PORT")
        .appendField(new Blockly.FieldTextInput("6377"), "PORT");
      this.setPreviousStatement(true, null);
      this.setNextStatement(true, null);
      this.setColour(30);
      this.setTooltip("");
      this.setHelpUrl("");
    }
  };

  Blockly.Blocks['remotexyrun'] = {
    init: function () {
      this.appendDummyInput()
        .appendField("RemoteXY : Running")
      //.appendField(new Blockly.FieldTextInput("0x42"), "ADDS");
      this.setInputsInline(true);
      this.setPreviousStatement(true, null);
      this.setNextStatement(true, null);
      this.setColour(30);
      this.setTooltip("");
      this.setHelpUrl("");
    }
  };

  Blockly.Blocks["dronebegin"] = {
    init: function () {

      this.appendDummyInput()
        .appendField(new Blockly.FieldImage(
          "https://static.thenounproject.com/png/415122-200.png",
          24,
          24,
          "*"))
        .appendField("Drone: Setup");
      this.appendDummyInput()
        .setAlign(Blockly.ALIGN_LEFT)
        .appendField("MPU6050 : Offset")
      this.appendDummyInput()
        .setAlign(Blockly.ALIGN_LEFT)
        .appendField("setAcc_X")
        .appendField(new Blockly.FieldTextInput("1624"), "setXAccelOffset");
      this.appendDummyInput()
        .setAlign(Blockly.ALIGN_LEFT)
        .appendField("setAcc_Y")
        .appendField(new Blockly.FieldTextInput("-589"), "setYAccelOffset");
      this.appendDummyInput()
        .setAlign(Blockly.ALIGN_LEFT)
        .appendField("setAcc_Z")
        .appendField(new Blockly.FieldTextInput("1272"), "setZAccelOffset");
        this.appendDummyInput()
        .setAlign(Blockly.ALIGN_LEFT)
        .appendField("setGyro_X")
        .appendField(new Blockly.FieldTextInput("64"), "setXGyroOffset");
      this.appendDummyInput()
        .setAlign(Blockly.ALIGN_LEFT)
        .appendField("setGyro_Y")
        .appendField(new Blockly.FieldTextInput("57"), "setYGyroOffset");
      this.appendDummyInput()
        .setAlign(Blockly.ALIGN_LEFT)
        .appendField("setGyro_Z")
        .appendField(new Blockly.FieldTextInput("85"), "setZGyroOffset");
      this.setPreviousStatement(true, null);
      this.setNextStatement(true, null);
      this.setColour(260);
      this.setTooltip("");
      this.setHelpUrl("");
    }
  };

  Blockly.Blocks['dronecal'] = {
    init: function () {
      this.appendDummyInput()
        .appendField("Drone : Calibration")
      //.appendField(new Blockly.FieldTextInput("0x42"), "ADDS");
      this.setInputsInline(true);
      this.setPreviousStatement(true, null);
      this.setNextStatement(true, null);
      this.setColour(220);
      this.setTooltip("");
      this.setHelpUrl("");
    }
  };

  Blockly.Blocks['dronerun'] = {
    init: function () {
      this.appendDummyInput()
        .appendField("Drone : Running")
      //.appendField(new Blockly.FieldTextInput("0x42"), "ADDS");
      this.setInputsInline(true);
      this.setPreviousStatement(true, null);
      this.setNextStatement(true, null);
      this.setColour(260);
      this.setTooltip("");
      this.setHelpUrl("");
    }
  };

  Blockly.Blocks["drone_readbat"] = {
		init: function () {
			this.appendDummyInput()
      .appendField("Drone : Batterry (mV)")
			this.setOutput(true, 'Number');
			this.setPreviousStatement(false);
			this.setNextStatement(false);
			this.setColour(150);
			this.setTooltip(Blockly.Msg.SENSOR_LDR_TOOLTIP);
			this.setHelpUrl(Blockly.Msg.SENSOR_LDR_HELPURL);
		}
	};
  
  Blockly.Blocks["sensor_tof"] = {
		init: function () {
			this.appendDummyInput()
				.appendField("Drone : Time-of-flight (cm)");
			this.setOutput(true, 'Number');
			this.setPreviousStatement(false);
			this.setNextStatement(false);
			this.setColour(340);
			this.setTooltip(Blockly.Msg.SENSOR_LDR_TOOLTIP);
			this.setHelpUrl(Blockly.Msg.SENSOR_LDR_HELPURL);
		}
	};

};