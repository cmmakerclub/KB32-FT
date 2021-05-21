module.exports = function (Blockly) {
	'use strict';
	// =============================================================================
	// MPU6050
	// =============================================================================
	Blockly.JavaScript['sensor_setmpu6050'] = function (block) {
		// TODO: Assemble JavaScript into code variable.
		var code =
		  `
		  mpu.begin();
	\n
	`;
		return code;
	  };

	Blockly.JavaScript['sensor_accmpu6050'] = function (block) {
		return [
			'mpu.Read(' + block.getFieldValue('OUTPUT') + ',' + 0 + ',' + 0 + ')',
			Blockly.JavaScript.ORDER_ATOMIC
		];
	};

	Blockly.JavaScript['sensor_gyrompu6050'] = function (block) {
		return [
			'mpu.Read(' + 0 + ',' + block.getFieldValue('OUTPUT') + ',' + 0 + ')',
			Blockly.JavaScript.ORDER_ATOMIC
		];
	};

	Blockly.JavaScript['sensor_tempmpu6050'] = function (block) {
		return [
			'(float)mpu.Read(' + 0 + ',' + 0 + ',' + 1 + ')/10',
			Blockly.JavaScript.ORDER_ATOMIC
		];
	};

	Blockly.JavaScript['sensor_kalAngle'] = function (block) {
		return [
			'(float)mpu.readAngle(' + block.getFieldValue('OUTPUT') + ')',
			Blockly.JavaScript.ORDER_ATOMIC
		];
	};

};