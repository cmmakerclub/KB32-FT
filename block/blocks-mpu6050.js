const dirIcon = Vue.prototype.$global.board.board_info.dir;
module.exports = function (Blockly) {
	'use strict';
	// =============================================================================
	// sensor
	// =============================================================================
	var sensor_colour = 10;
	// var sensor_colour="#CAC745";

	Blockly.Blocks['sensor_setmpu6050'] = {
		init: function () {
		  this.appendDummyInput()
			.appendField("MPU6050 : Setup")
		  //.appendField(new Blockly.FieldTextInput("0x42"), "ADDS");
		  this.setInputsInline(true);
		  this.setPreviousStatement(true, null);
		  this.setNextStatement(true, null);
		  this.setColour(sensor_colour);
		  this.setTooltip("");
		  this.setHelpUrl("");
		}
	  };

	Blockly.Blocks['sensor_accmpu6050'] = {
		init: function () {
			this.appendDummyInput()
				.appendField(new Blockly.FieldImage(`file:///${dirIcon}/static/icons/mpu.png`, 20, 20, "*"))
				.appendField("Accelerometer")
				.appendField(new Blockly.FieldDropdown([
					["X", "1"],
					["Y", "2"],
					["Z", "3"],

				]), 'OUTPUT')
				.appendField("Axis");
			this.setOutput(true, ["int16_t", "Number"]);
			this.setInputsInline(true);
			this.setPreviousStatement(null);
			this.setNextStatement(null);
			this.setColour(sensor_colour);
			this.setTooltip("");
			this.setHelpUrl("");
		}
	};

	Blockly.Blocks['sensor_gyrompu6050'] = {
		init: function () {
			this.appendDummyInput()
				.appendField(new Blockly.FieldImage(`file:///${dirIcon}/static/icons/mpu.png`, 20, 20, "*"))
				.appendField("Gyroscope")
				.appendField(new Blockly.FieldDropdown([
					["X", "1"],
					["Y", "2"],
					["Z", "3"],

				]), 'OUTPUT')
				.appendField("Axis");
			this.setOutput(true, ["int16_t", "Number"]);
			this.setInputsInline(true);
			this.setPreviousStatement(null);
			this.setNextStatement(null);
			this.setColour(sensor_colour);
			this.setTooltip("");
			this.setHelpUrl("");
		}
	};

	Blockly.Blocks['sensor_tempmpu6050'] = {
		init: function () {
			this.appendDummyInput()
				.appendField(new Blockly.FieldImage(`file:///${dirIcon}/static/icons/mpu.png`, 20, 20, "*"))
				.appendField("Temperature")
			this.setOutput(true, ["float", "Number"]);
			this.setPreviousStatement(false);
			this.setNextStatement(false);
			this.setColour(sensor_colour);
			this.setTooltip(Blockly.Msg.SENSOR_LDR_TOOLTIP);
			this.setHelpUrl(Blockly.Msg.SENSOR_LDR_HELPURL);
		}
	};

	Blockly.Blocks['sensor_kalAngle'] = {
		init: function () {
			this.appendDummyInput()
				.appendField(new Blockly.FieldImage(`file:///${dirIcon}/static/icons/mpu.png`, 20, 20, "*"))
				.appendField("Angle")
				.appendField(new Blockly.FieldDropdown([
					["X", "0"],
					["Y", "1"],

				]), 'OUTPUT')
				.appendField("Axis");
			this.setOutput(true, ["float", "Number"]);
			this.setInputsInline(true);
			this.setPreviousStatement(null);
			this.setNextStatement(null);
			this.setColour(sensor_colour);
			this.setTooltip("");
			this.setHelpUrl("");
		}
	};


};