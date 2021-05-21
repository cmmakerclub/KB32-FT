const dirIcon = Vue.prototype.$global.board.board_info.dir;
module.exports = function (Blockly) {
	'use strict';
	// =============================================================================
	// sensor
	// =============================================================================
	var sensor_colour = Blockly.Msg.SENSOR_HUE;
	// var sensor_colour="#CAC745";

	Blockly.Blocks["sensor_lm73"] = {
		init: function () {
			this.appendDummyInput()
				.appendField(new Blockly.FieldImage("/static/block_icons/sot23-6_1.png", 20, 20, "*"))
				.appendField(Blockly.Msg.SENSOR_LM73_TITLE);
			this.setOutput(true, ["float", "Number"]);
			this.setPreviousStatement(false);
			this.setNextStatement(false);
			this.setColour(sensor_colour);
			this.setTooltip(Blockly.Msg.SENSOR_LM73_TOOLTIP);
			this.setHelpUrl(Blockly.Msg.SENSOR_LM73_HELPURL);
		}
	};

	Blockly.Blocks["sensor_ldr"] = {
		init: function () {
			this.appendDummyInput()
				.appendField(new Blockly.FieldImage("/static/block_icons/ldr1.png", 20, 20, "*"))
				.appendField(Blockly.Msg.SENSOR_LDR_TITLE);
			this.setOutput(true, 'Number');
			this.setPreviousStatement(false);
			this.setNextStatement(false);
			this.setColour(sensor_colour);
			this.setTooltip(Blockly.Msg.SENSOR_LDR_TOOLTIP);
			this.setHelpUrl(Blockly.Msg.SENSOR_LDR_HELPURL);
		}
	};

	Blockly.Blocks["sensor_lux"] = {
		init: function () {
			this.appendDummyInput()
				.appendField(new Blockly.FieldImage(`file:///${dirIcon}/static/icons/lux.png`, 20, 20, "*"))
				.appendField("LUX Meter Auto Gain");
			this.setOutput(true, 'Number');
			this.setPreviousStatement(false);
			this.setNextStatement(false);
			this.setColour(340);
			this.setTooltip(Blockly.Msg.SENSOR_LDR_TOOLTIP);
			this.setHelpUrl(Blockly.Msg.SENSOR_LDR_HELPURL);
		}
	};

	Blockly.Blocks["sensor_luxlowgain"] = {
		init: function () {
			this.appendDummyInput()
				.appendField(new Blockly.FieldImage(`file:///${dirIcon}/static/icons/lux.png`, 20, 20, "*"))
				.appendField("LUX Meter LOW Gain 0-9,660 lx");
			this.setOutput(true, 'Number');
			this.setPreviousStatement(false);
			this.setNextStatement(false);
			this.setColour(340);
			this.setTooltip(Blockly.Msg.SENSOR_LDR_TOOLTIP);
			this.setHelpUrl(Blockly.Msg.SENSOR_LDR_HELPURL);
		}
	};

	Blockly.Blocks["sensor_luxhighgain"] = {
		init: function () {
			this.appendDummyInput()
				.appendField(new Blockly.FieldImage(`file:///${dirIcon}/static/icons/lux.png`, 20, 20, "*"))
				.appendField("LUX Meter HIGH Gain 0-966 lx");
			this.setOutput(true, 'Number');
			this.setPreviousStatement(false);
			this.setNextStatement(false);
			this.setColour(340);
			this.setTooltip(Blockly.Msg.SENSOR_LDR_TOOLTIP);
			this.setHelpUrl(Blockly.Msg.SENSOR_LDR_HELPURL);
		}
	};

	Blockly.Blocks["sensor_switch1"] = {
		init: function () {
			this.appendDummyInput()
				.appendField(new Blockly.FieldImage("/static/block_icons/sw12x12.png", 20, 20, "*"))
				.appendField(Blockly.Msg.SENSOR_SWITCH1_TITLE);
			this.setOutput(true, ['Number', 'Boolean']);
			this.setPreviousStatement(false);
			this.setNextStatement(false);
			this.setColour(sensor_colour);
			this.setTooltip(Blockly.Msg.SENSOR_SWITCH1_TOOLTIP);
			this.setHelpUrl(Blockly.Msg.SENSOR_SWITCH1_HELPURL);
		}
	};

	Blockly.Blocks["sensor_switch2"] = {
		init: function () {
			this.appendDummyInput()
				.appendField(new Blockly.FieldImage("/static/block_icons/sw12x12.png", 20, 20, "*"))
				.appendField(Blockly.Msg.SENSOR_SWITCH2_TITLE);
			this.setOutput(true, ['Number', 'Boolean']);
			this.setPreviousStatement(false);
			this.setNextStatement(false);
			this.setColour(sensor_colour);
			this.setTooltip(Blockly.Msg.SENSOR_SWITCH2_TOOLTIP);
			this.setHelpUrl(Blockly.Msg.SENSOR_SWITCH2_HELPURL);
		}
	};

};