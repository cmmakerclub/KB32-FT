const dirIcon = Vue.prototype.$global.board.board_info.dir;
module.exports = function (Blockly) {
  "use strict";

  Blockly.Blocks["remote_xy_begin"] = {
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



	Blockly.Blocks['remote_xy_get'] = {
		init: function () {
			this.appendDummyInput()
				.appendField("Read Data")
				.appendField(new Blockly.FieldDropdown([
					["Joy1_x", "1"],
					["Joy1_y", "2"],
					["Joy2_x", "3"],
					["Joy2_y", "4"],
					["Switch", "5"],
					["WiFi", "6"],
				]), 'OUTPUT')
				.appendField("");

			this.setOutput(true, "Number");
			this.setInputsInline(true);
			this.setPreviousStatement(null);
			this.setNextStatement(null);
			this.setColour(350);
			this.setTooltip("");
			this.setHelpUrl("");
		}
	};

};