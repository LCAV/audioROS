/**
 * @fileoverview Generating AESL for Thymio blocks.
 * @author fabian@hahn.graphics (Fabian Hahn)
 */
'use strict';

goog.provide('Blockly.AESL.thymio');

goog.require('Blockly.AESL');

function hexToRgb(hex)
{
	var result = /^#?([a-f\d]{2})([a-f\d]{2})([a-f\d]{2})$/i.exec(hex);
	return result ? {
		r : parseInt(result[1], 16),
		g : parseInt(result[2], 16),
		b : parseInt(result[3], 16)
	} : null;
}

function rgbToAesl(rgb)
{
	return rgb ? {
		r : Math.round(rgb.r / 8),
		g : Math.round(rgb.g / 8),
		b : Math.round(rgb.b / 8)
	} : null;
}

function hexToAesl(hex)
{
	return rgbToAesl(hexToRgb(hex));
}

Blockly.AESL['thymio_when'] = function(block)
{
	var condition = Blockly.AESL.valueToCode(block, 'WHEN', Blockly.AESL.ORDER_NONE);
	var branch = Blockly.AESL.statementToCode(block, 'DO');
	
	var code = 'when ' + condition + ' do\n' + branch + 'end\n';
	return code;
};

Blockly.AESL['thymio_for'] = function(block)
{
	var iterator = Blockly.AESL.variableDB_.getName(block.getFieldValue('ITER'), Blockly.Variables.NAME_TYPE);
	var from = parseInt(block.getFieldValue('FROM'));
	var to = parseInt(block.getFieldValue('TO'));
	var branch = Blockly.AESL.statementToCode(block, 'DO');
	
	var code = 'for ' + iterator + ' in ' + from + ':' + to + ' do\n' + branch + 'end\n';
	return code;
};

Blockly.AESL['thymio_subroutine_define'] = function(block)
{
	// Define a procedure with a return value.
	var funcName = Blockly.AESL.variableDB_.getName(block.getFieldValue('NAME'), Blockly.Procedures.NAME_TYPE);
	var branch = Blockly.AESL.statementToCode(block, 'STACK');
	
	if(branch.indexOf('\tcallsub ') != -1) {
		// AESL does allow subroutines to call other subroutines, but they need to be defined in the correct
		// order. Since we currently no way to track this in Blockly, let's just disallow calling other subs
		// altogether
		Blockly.AESL.addSubroutine(funcName, '');
		return null;
	}

	// Ignore input arguments - AESL subroutines cannot receive or return values
	
	Blockly.AESL.addSubroutine(funcName, branch);
	return null;
};

Blockly.AESL['thymio_event'] = function(block)
{
	var event = block.getFieldValue('EVENT');
	var handler = Blockly.AESL.statementToCode(block, 'HANDLER');
	
	Blockly.AESL.addEventHandler(event, handler);
	return null;
};

Blockly.AESL['thymio_event_button'] = function(block)
{
	var button = block.getFieldValue('BUTTON');
	var mode = block.getFieldValue('MODE');
	var handler = Blockly.AESL.statementToCode(block, 'HANDLER');
	
	var code = '\twhen ' + button + ' == ' + (mode == 'PRESS' ? '1' : '0') + ' do\n';
	
	if(handler.length > 0) {
		code += Blockly.AESL.prefixLines(handler, '\t');
	}
	
	code += '\tend\n';

	Blockly.AESL.addEventHandler(button, code);
	return null;
};

Blockly.AESL['thymio_event_prox'] = function(block)
{
	var sensor = block.getFieldValue('SENSOR');
	var mode = block.getFieldValue('MODE');
	var handler = Blockly.AESL.statementToCode(block, 'HANDLER');
	
	var condition = '';
	if(mode == 'PROX') {
		condition = ' > 2000';
	} else {
		condition = ' < 1000';
	}
	
	var code = '\twhen ' + sensor + condition + ' do\n';
	
	if(handler.length > 0) {
		code += Blockly.AESL.prefixLines(handler, '\t');
	}
	
	code += '\tend\n';

	Blockly.AESL.addEventHandler('prox', code);
	return null;
};

Blockly.AESL['thymio_event_prox_ground'] = function(block)
{
	var sensor = block.getFieldValue('SENSOR');
	var mode = block.getFieldValue('MODE');
	var handler = Blockly.AESL.statementToCode(block, 'HANDLER');
	
	var condition = '';
    if(mode == 'WHITE' || mode == 'PROX') {
		condition = ' > 450';
	} else {
		condition = ' < 400';
	}
	
	var code = '\twhen ' + sensor + condition + ' do\n';
	
	if(handler.length > 0) {
		code += Blockly.AESL.prefixLines(handler, '\t');
	}
	
	code += '\tend\n';

	Blockly.AESL.addEventHandler('prox', code);
	return null;
};

Blockly.AESL['thymio_event_shock'] = function(block)
{
	var handler = Blockly.AESL.statementToCode(block, 'HANDLER');
	
	Blockly.AESL.addEventHandler('tap', handler);
	return null;
};

Blockly.AESL['thymio_event_timer'] = Blockly.AESL['thymio_event'];
Blockly.AESL['thymio_event_sound'] = Blockly.AESL['thymio_event'];
Blockly.AESL['thymio_event_acc'] = Blockly.AESL['thymio_event'];
Blockly.AESL['thymio_event_receive'] = Blockly.AESL['thymio_event'];
Blockly.AESL['thymio_event_update'] = Blockly.AESL['thymio_event'];

Blockly.AESL['thymio_led'] = function(block)
{
	var led = block.getFieldValue('LED');
	var color = hexToAesl(block.getFieldValue('COLOR'));

	var code = 'call ' + led + '(' + color.r + ',' + color.g + ',' + color.b + ')\n';
	return code;
};

Blockly.AESL['thymio_led_rgb'] = function(block)
{
	var led = block.getFieldValue('LED');
	var red = Blockly.AESL.valueToCode(block, 'RED', Blockly.AESL.ORDER_NONE) || '0';
	var green = Blockly.AESL.valueToCode(block, 'GREEN', Blockly.AESL.ORDER_NONE) || '0';
	var blue = Blockly.AESL.valueToCode(block, 'BLUE', Blockly.AESL.ORDER_NONE) || '0';

	var code = 'call ' + led + '(' + red + ',' + green + ',' + blue + ')\n';
	return code;
};

Blockly.AESL['thymio_led_circle'] = function(block)
{
	var circle0 = Blockly.AESL.valueToCode(block, 'CIRCLE0', Blockly.AESL.ORDER_NONE) || '0';
	var circle1 = Blockly.AESL.valueToCode(block, 'CIRCLE1', Blockly.AESL.ORDER_NONE) || '0';
	var circle2 = Blockly.AESL.valueToCode(block, 'CIRCLE2', Blockly.AESL.ORDER_NONE) || '0';
	var circle3 = Blockly.AESL.valueToCode(block, 'CIRCLE3', Blockly.AESL.ORDER_NONE) || '0';
	var circle4 = Blockly.AESL.valueToCode(block, 'CIRCLE4', Blockly.AESL.ORDER_NONE) || '0';
	var circle5 = Blockly.AESL.valueToCode(block, 'CIRCLE5', Blockly.AESL.ORDER_NONE) || '0';
	var circle6 = Blockly.AESL.valueToCode(block, 'CIRCLE6', Blockly.AESL.ORDER_NONE) || '0';
	var circle7 = Blockly.AESL.valueToCode(block, 'CIRCLE7', Blockly.AESL.ORDER_NONE) || '0';

	var code = 'call leds.circle(' + circle0 + ',' + circle1 + ',' + circle2 + ',' + circle3 + ',' + circle4 + ',' + circle5 + ',' + circle6 + ',' + circle7 + ')\n';
	return code;
};

Blockly.AESL['thymio_led_prox'] = function(block)
{
	var prox0 = Blockly.AESL.valueToCode(block, 'PROX0', Blockly.AESL.ORDER_NONE) || '0';
	var prox1 = Blockly.AESL.valueToCode(block, 'PROX1', Blockly.AESL.ORDER_NONE) || '0';
	var prox2 = Blockly.AESL.valueToCode(block, 'PROX2', Blockly.AESL.ORDER_NONE) || '0';
	var prox3 = Blockly.AESL.valueToCode(block, 'PROX3', Blockly.AESL.ORDER_NONE) || '0';
	var prox4 = Blockly.AESL.valueToCode(block, 'PROX4', Blockly.AESL.ORDER_NONE) || '0';
	var prox5 = Blockly.AESL.valueToCode(block, 'PROX5', Blockly.AESL.ORDER_NONE) || '0';
	var prox6 = Blockly.AESL.valueToCode(block, 'PROX6', Blockly.AESL.ORDER_NONE) || '0';
	var prox7 = Blockly.AESL.valueToCode(block, 'PROX7', Blockly.AESL.ORDER_NONE) || '0';

	var code = 'call leds.prox.h(' + prox0 + ',' + prox1 + ',' + prox2 + ',' + prox3 + ',' + prox4 + ',' + prox5 + ',' + prox6 + ',' + prox7 + ')\n';
	return code;
};

Blockly.AESL['thymio_led_prox_ground'] = function(block)
{
	var prox0 = Blockly.AESL.valueToCode(block, 'PROX0', Blockly.AESL.ORDER_NONE) || '0';
	var prox1 = Blockly.AESL.valueToCode(block, 'PROX1', Blockly.AESL.ORDER_NONE) || '0';

	var code = 'call leds.prox.v(' + prox0 + ',' + prox1 + ')\n';
	return code;
};

Blockly.AESL['thymio_led_button'] = function(block)
{
	var forward = Blockly.AESL.valueToCode(block, 'FORWARD', Blockly.AESL.ORDER_NONE) || '0';
	var right = Blockly.AESL.valueToCode(block, 'RIGHT', Blockly.AESL.ORDER_NONE) || '0';
	var backward = Blockly.AESL.valueToCode(block, 'BACKWARD', Blockly.AESL.ORDER_NONE) || '0';
	var left = Blockly.AESL.valueToCode(block, 'LEFT', Blockly.AESL.ORDER_NONE) || '0';

	var code = 'call leds.buttons(' + forward + ',' + right + ',' + backward + ',' + left + ')\n';
	return code;
};

Blockly.AESL['thymio_led_temperature'] = function(block)
{
	var red = Blockly.AESL.valueToCode(block, 'RED', Blockly.AESL.ORDER_NONE) || '0';
	var blue = Blockly.AESL.valueToCode(block, 'BLUE', Blockly.AESL.ORDER_NONE) || '0';

	var code = 'call leds.temperature(' + red + ',' + blue + ')\n';
	return code;
};

Blockly.AESL['thymio_led_rc_sound'] = function(block)
{
	var led = block.getFieldValue('LED');
	var intensity = Blockly.AESL.valueToCode(block, 'INTENSITY', Blockly.AESL.ORDER_NONE) || '0';

	var code = 'call ' + led + '(' + intensity + ')\n';
	return code;
};

Blockly.AESL['thymio_led_off'] = function(block)
{
	var led = block.getFieldValue('LED');
	
	var offstr = '';
	
	if(led == 'leds.top' || led == 'leds.bottom.left' || led == 'leds.bottom.right') {
		offstr = '0,0,0';
	} else if(led == 'leds.circle' || led == 'leds.prox.h') {
		offstr = '0,0,0,0,0,0,0,0';
	} else if(led == 'leds.prox.v' || led == 'leds.temperature') {
		offstr = '0,0';
	} else if(led == 'leds.rc' || led == 'leds.sound') {
		offstr = '0';
	} else if(led == 'leds.buttons') {
		offstr = '0,0,0,0';
	}

	var code = 'call ' + led + '(' + offstr + ')\n';
	return code;
};

Blockly.AESL['thymio_sound_system'] = function(block)
{
	var sound = block.getFieldValue('SOUND');

	var code = 'call sound.system(' + sound + ')\n';
	return code;
};

Blockly.AESL['thymio_sound_note'] = function(block)
{
	var freq = Blockly.AESL.valueToCode(block, 'FREQ', Blockly.AESL.ORDER_NONE) || '0';
	var duration = Blockly.AESL.valueToCode(block, 'DURATION', Blockly.AESL.ORDER_NONE) || '0';

	var code = 'call sound.freq(' + freq + ',' + duration + ')\n';
	return code;
};

Blockly.AESL['thymio_sound_stop'] = function(block)
{
	var code = 'call sound.system(-1)\n';
	return code;
};

Blockly.AESL['thymio_button_pressed'] = function(block)
{
	var button = block.getFieldValue('BUTTON');
	return [button + ' == 1', Blockly.AESL.ORDER_LOGICAL_AND];
};

Blockly.AESL['thymio_prox_check'] = function(block)
{
	var sensor = block.getFieldValue('SENSOR');
	var mode = block.getFieldValue('MODE');
	
	var condition = '';
	if(mode == 'PROX') {
		condition = ' > 2000';
	} else {
		condition = ' < 1000';
	}
	
	return [sensor + condition, Blockly.AESL.ORDER_CONDITION];
};

Blockly.AESL['thymio_prox_ground_check'] = function(block)
{
	var sensor = block.getFieldValue('SENSOR');
	var mode = block.getFieldValue('MODE');
	
	var condition = '';
    if(mode == 'WHITE' || mode == 'PROX') {
		condition = ' > 450';
	} else {
		condition = ' < 400';
	}
	
	return [sensor + condition, Blockly.AESL.ORDER_CONDITION];
};

Blockly.AESL['thymio_sensor'] = function(block)
{
	var sensor = block.getFieldValue('SENSOR');
	return [sensor, Blockly.AESL.ORDER_ATOMIC];
};

Blockly.AESL['thymio_sensor_temperature'] = function(block)
{
	return ['temperature', Blockly.AESL.ORDER_ATOMIC];
};

Blockly.AESL['thymio_sensor_mic'] = function(block)
{
	return ['mic.intensity', Blockly.AESL.ORDER_ATOMIC];
};

Blockly.AESL['thymio_sensor_comm'] = function(block)
{
	return ['prox.comm.rx', Blockly.AESL.ORDER_ATOMIC];
};

Blockly.AESL['thymio_sensor_prox'] = Blockly.AESL['thymio_sensor'];
Blockly.AESL['thymio_sensor_motor'] = Blockly.AESL['thymio_sensor'];
Blockly.AESL['thymio_sensor_acc'] = Blockly.AESL['thymio_sensor'];
Blockly.AESL['thymio_sensor_rc'] = Blockly.AESL['thymio_sensor'];

Blockly.AESL['thymio_motors_start'] = function(block)
{
	var command = block.getFieldValue('COMMAND');
	var speed = Blockly.AESL.valueToCode(block, 'SPEED', Blockly.AESL.ORDER_ASSIGNMENT) || '0';
	
	var leftTarget = 0;
	var rightTarget = 0;
	
	if(command == 'FORWARD') {
		leftTarget = speed;
		rightTarget = speed;
	} else if(command == 'BACKWARD') {
		leftTarget = -speed;
		rightTarget = -speed;
	} else if(command == 'TURNLEFT') {
		leftTarget = 0;
		rightTarget = speed;
	} else if(command == 'TURNRIGHT') {
		leftTarget = speed;
		rightTarget = 0;
	} else if(command == 'TURNBACKWARDLEFT') {
		leftTarget = 0;
		rightTarget = -speed;
	} else if(command == 'TURNBACKWARDRIGHT') {
		leftTarget = -speed;
		rightTarget = 0;
	} else if(command == 'SPINCCW') {
		leftTarget = -speed;
		rightTarget = speed;
	} else if(command == 'SPINCW') {
		leftTarget = speed;
		rightTarget = -speed;
	}

	var code = 'motor.left.target = ' + leftTarget + '\n' +
		'motor.right.target = ' + rightTarget + '\n';
	return code;
};

Blockly.AESL['thymio_motors_stop'] = function(block)
{
	var code = 'motor.left.target = 0\n' +
		'motor.right.target = 0\n';
	return code;
};


Blockly.AESL['thymio_actuator'] = function(block)
{
	var variable = block.getFieldValue('VARIABLE');
	var value = Blockly.AESL.valueToCode(block, 'VALUE', Blockly.AESL.ORDER_ASSIGNMENT) || '0';

	var code = variable + ' = ' + value + '\n';
	return code;
};

Blockly.AESL['thymio_actuator_mic'] = function(block)
{
	var value = Blockly.AESL.valueToCode(block, 'VALUE', Blockly.AESL.ORDER_ASSIGNMENT) || '0';

	var code = 'mic.threshold = ' + value + '\n';
	return code;
};

Blockly.AESL['thymio_actuator_comm'] = function(block)
{
	var value = Blockly.AESL.valueToCode(block, 'VALUE', Blockly.AESL.ORDER_ASSIGNMENT) || '0';

	var code = 'prox.comm.tx = ' + value + '\n';
	return code;
};

Blockly.AESL['thymio_actuator_timer'] = Blockly.AESL['thymio_actuator'];
Blockly.AESL['thymio_actuator_motor'] = Blockly.AESL['thymio_actuator'];

Blockly.AESL['thymio_variable_get'] = function(block)
{
	// Variable getter.
	var code = '@variable:' + Blockly.AESL.variableDB_.getName(block.getFieldValue('VAR'), Blockly.Variables.NAME_TYPE) + '@';
	
	return [code, Blockly.AESL.ORDER_ATOMIC];
};

Blockly.AESL['thymio_variable_set'] = function(block)
{
	var varName = Blockly.AESL.variableDB_.getName(block.getFieldValue('VAR'), Blockly.Variables.NAME_TYPE);
	
	// AESL only allows int variables, so check if our input has the correct type
	var valueBlock = this.getInputTargetBlock('VALUE');
	
	var argument0;	
    if(!Blockly.AESL.checkValueBlockType(valueBlock)) {
    	argument0 = '0';
    } else {
    	argument0 = Blockly.AESL.valueToCode(block, 'VALUE', Blockly.AESL.ORDER_ASSIGNMENT) || '0';
    }
	
	// Variable setter.
	return '@variable:' + varName + '@ = ' + argument0 + '\n';
};


Blockly.AESL['thymio_declare_array'] = function(block)
{
	var variable = Blockly.AESL.variableDB_.getName(block.getFieldValue('VAR'), Blockly.Variables.NAME_TYPE);	
	var size = parseInt(block.getFieldValue('SIZE'));
	
	if(size < 1) {
		size = 1;
	}
	
	Blockly.AESL.arrays[variable] = size;
	return null;
};

Blockly.AESL['thymio_set_array'] = function(block)
{
	var variable = Blockly.AESL.variableDB_.getName(block.getFieldValue('VAR'), Blockly.Variables.NAME_TYPE);
	var index = Blockly.AESL.valueToCode(block, 'INDEX', Blockly.AESL.ORDER_INDEX) || '0';
	var value = Blockly.AESL.valueToCode(block, 'VALUE', Blockly.AESL.ORDER_ASSIGNMENT) || '0';
	
	var code = variable + '[' + index + '] = ' + value + '\n';
	
	if(!(variable in Blockly.AESL.arrays)) { // if no array with this name has been defined yet, initialize one
		Blockly.AESL.arrays[variable] = 1;
	}
	
	return code;
};

Blockly.AESL['thymio_get_array'] = function(block)
{
	var variable = Blockly.AESL.variableDB_.getName(block.getFieldValue('VAR'), Blockly.Variables.NAME_TYPE);
	var index = Blockly.AESL.valueToCode(block, 'INDEX', Blockly.AESL.ORDER_NONE) || '0';

	var code = variable + '[' + index + ']';
	
	if(!(variable in Blockly.AESL.arrays)) { // if no array with this name has been defined yet, initialize one
		Blockly.AESL.arrays[variable] = 1;
	}
	
	return [code, Blockly.AESL.ORDER_INDEX];
};

Blockly.AESL['thymio_compare'] = function(block)
{
	var blockA = this.getInputTargetBlock('A');
    var blockB = this.getInputTargetBlock('B');
	
	// Comparison operator.
	var OPERATORS = {
		'EQ' : '==',
		'NEQ' : '!=',
		'LT' : '<',
		'LTE' : '<=',
		'GT' : '>',
		'GTE' : '>='
	};
	var operator = OPERATORS[block.getFieldValue('OP')];
	var order = Blockly.AESL.ORDER_CONDITION;
	var argument0 = Blockly.AESL.valueToCode(block, 'A', order) || '0';
	var argument1 = Blockly.AESL.valueToCode(block, 'B', order) || '0';
	var code = argument0 + ' ' + operator + ' ' + argument1;
	return [code, order];
};

Blockly.AESL['thymio_arithmetic'] = function(block)
{
	var operator = block.getFieldValue('OP');
	
	var order = Blockly.AESL.ORDER_MULT;
		
	if(operator == '+' || operator == '-') {
		order = Blockly.AESL.ORDER_ADD;
	}
	
	var argument0 = Blockly.AESL.valueToCode(block, 'A', order) || '0';
	var argument1 = Blockly.AESL.valueToCode(block, 'B', order) || '0';
	
	var code = argument0 + ' ' + operator + ' ' + argument1;
	return [code, order];
};

Blockly.AESL['thymio_binary'] = function(block)
{
	var operator = block.getFieldValue('OP');
	
	var order;
	if(operator == '<<' || operator == '>>') {
		order = Blockly.AESL.ORDER_SHIFT;
	} else if(operator == '&') {
		order = Blockly.AESL.ORDER_BINARY_AND;
	} else if(operator == '|') {
		order = Blockly.AESL.ORDER_BINARY_OR;
	} else if(operator == '^') {
		order = Blockly.AESL.ORDER_BINARY_XOR;
	}
	
	var argument0 = Blockly.AESL.valueToCode(block, 'A', order) || '0';
	var argument1 = Blockly.AESL.valueToCode(block, 'B', order) || '0';
	
	var code = argument0 + ' ' + operator + ' ' + argument1;
	return [code, order];
};

Blockly.AESL['thymio_unary'] = function(block)
{
	var operator = block.getFieldValue('OP');
	
	var order;
	if(operator == '-') {
		order = Blockly.AESL.ORDER_MINUS;
	} else if(operator == 'abs') {
		order = Blockly.AESL.ORDER_ABS;
	} else if(operator == '~') {
		order = Blockly.AESL.ORDER_BINARY_NOT;
	}
	
	var value = Blockly.AESL.valueToCode(block, 'VALUE', order) || '0';
	
	var code = operator + (operator == 'abs' ? ' ' : '') + value;
	return [code, order];
};

Blockly.AESL['thymio_communication'] = function(block)
{
	var mode = block.getFieldValue('MODE');
	
	var code;
	if(mode == 'ENABLE') {	
		code = 'call prox.comm.enable(1)\n';
	} else {
		code = 'call prox.comm.enable(0)\n';
	}
	
	return code;
};

