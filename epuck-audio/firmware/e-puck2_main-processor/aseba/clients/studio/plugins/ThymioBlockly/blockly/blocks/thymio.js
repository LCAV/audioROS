/**
 * @fileoverview Blocks for Thymio.
 * @author fabian@hahn.graphics (Fabian Hahn)
 */
'use strict';

goog.provide('Blockly.Blocks.thymio');

goog.require('Blockly.Blocks');

/**
 * Common HSV hue for all blocks in this category.
 */
Blockly.Blocks.thymio.ACTUATORS_HUE = 60;
Blockly.Blocks.thymio.LEDS_HUE = 160;
Blockly.Blocks.thymio.SENSORS_HUE = 90;
Blockly.Blocks.thymio.EVENTS_HUE = 10;

/*
Blockly.Blocks.colour.HUE = 20;
Blockly.Blocks.loops.HUE = 120;
Blockly.Blocks.texts.HUE = 160;
Blockly.Blocks.logic.HUE = 210;
Blockly.Blocks.math.HUE = 230;
Blockly.Blocks.lists.HUE = 260;
Blockly.Blocks.procedures.HUE = 290;
Blockly.Blocks.variables.HUE = 330;
*/

Blockly.Blocks['thymio_when'] = {
	/**
	 * Block for Thymio when conditions.
	 * 
	 * @this Blockly.Block
	 */
	init : function()
	{
		this.setColour(Blockly.Blocks.logic.HUE);
		this.setHelpUrl(Blockly.Msg.THYMIO_WHEN_HELPURL);
		this.setTooltip(Blockly.Msg.THYMIO_WHEN_TOOLTIP);
		this.setPreviousStatement(true);
	    this.setNextStatement(true);
		
		this.appendValueInput('WHEN').setCheck('Boolean').appendField(Blockly.Msg.THYMIO_WHEN_WHEN);
		this.appendStatementInput('DO').appendField(Blockly.Msg.THYMIO_WHEN_DO);
	}
};

Blockly.Blocks['thymio_for'] = {
	/**
	 * Block for Thymio count loops.
	 * 
	 * @this Blockly.Block
	 */
	init : function()
	{
		this.setColour(Blockly.Blocks.loops.HUE);
		this.setHelpUrl(Blockly.Msg.THYMIO_FOR_HELPURL);
		this.setTooltip(Blockly.Msg.THYMIO_FOR_TOOLTIP);
		this.setPreviousStatement(true);
		this.setNextStatement(true);

		var variableField = new Blockly.FieldVariable('');
		var fromField = new Blockly.FieldTextInput('1', Blockly.FieldTextInput.numberValidator);
		var toField = new Blockly.FieldTextInput('10', Blockly.FieldTextInput.numberValidator);

		this.appendDummyInput().appendField(Blockly.Msg.THYMIO_FOR_FOR).appendField(variableField, 'ITER').appendField(Blockly.Msg.THYMIO_FOR_FROM).appendField(fromField, 'FROM').appendField(Blockly.Msg.THYMIO_FOR_TO).appendField(toField, 'TO');
		this.appendStatementInput('DO').appendField(Blockly.Msg.THYMIO_FOR_DO);
	},
	/**
	 * Return all variables referenced by this block.
	 * 
	 * @return {!Array.<string>} List of variable names.
	 * @this Blockly.Block
	 */
	getVars : function()
	{
		return [this.getFieldValue('ITER')];
	},
	/**
	 * Notification that a variable is renaming. If the name matches one of this
	 * block's variables, rename it.
	 * 
	 * @param {string} oldName Previous name of variable.
	 * @param {string} newName Renamed variable.
	 * @this Blockly.Block
	 */
	renameVar : function(oldName, newName)
	{
		if(Blockly.Names.equals(oldName, this.getFieldValue('ITER'))) {
			this.setFieldValue(newName, 'ITER');
		}
	}
};

Blockly.Blocks['thymio_subroutine_define'] = {
	/**
	 * Block to define Thymio subroutines
	 * 
	 * @this Blockly.Block
	 */
	init : function()
	{
		this.setColour(Blockly.Blocks.procedures.HUE);
		this.setHelpUrl(Blockly.Msg.THYMIO_SUBROUTINE_DEFINE_HELPURL);
		this.setTooltip(Blockly.Msg.THYMIO_SUBROUTINE_DEFINE_TOOLTIP);

		var nameField = new Blockly.FieldTextInput('name', Blockly.Procedures.rename);

		this.appendDummyInput().appendField(Blockly.Msg.THYMIO_SUBROUTINE_DEFINE_SUBROUTINE).appendField(nameField, 'NAME');
		this.appendStatementInput('STACK');
	},
	/**
	 * Dispose of any callers.
	 * 
	 * @this Blockly.Block
	 */
	dispose : function()
	{
		var name = this.getFieldValue('NAME');
		Blockly.Procedures.disposeCallers(name, this.workspace);
		// Call parent's destructor.
		this.constructor.prototype.dispose.apply(this, arguments);
	},

	/**
	 * Return the signature of this procedure definition.
	 * 
	 * @return {!Array} Tuple containing three elements: - the name of the
	 *         defined procedure, - a list of all its arguments, - that it DOES
	 *         NOT have a return value.
	 * @this Blockly.Block
	 */
	getProcedureDef : function()
	{
		return [this.getFieldValue('NAME'), [], false];
	},
	callType_ : 'procedures_callnoreturn'
};

Blockly.Blocks['thymio_event_button'] = {
	/**
	 * Block for Thymio button event.
	 * 
	 * @this Blockly.Block
	 */
	init : function()
	{
		this.jsonInit({
			"message0" : Blockly.Msg.THYMIO_EVENT_BUTTON,
			"args0" : [{
				"type" : "field_dropdown",
				"name" : "BUTTON",
				"options" : [
					[Blockly.Msg.THYMIO_EVENT_BUTTON_CENTER, 'button.center'],
					[Blockly.Msg.THYMIO_EVENT_BUTTON_FORWARD, 'button.forward'],
					[Blockly.Msg.THYMIO_EVENT_BUTTON_BACKWARD, 'button.backward'],
					[Blockly.Msg.THYMIO_EVENT_BUTTON_LEFT, 'button.left'],
					[Blockly.Msg.THYMIO_EVENT_BUTTON_RIGHT, 'button.right']]
			}, {
				"type" : "field_dropdown",
				"name" : "MODE",
				"options" : [
					[Blockly.Msg.THYMIO_EVENT_BUTTON_PRESS, 'PRESS'],
					[Blockly.Msg.THYMIO_EVENT_BUTTON_RELEASE, 'RELEASE']]
			}]
		});
		
		this.setColour(Blockly.Blocks.thymio.EVENTS_HUE);
		this.setHelpUrl(Blockly.Msg.THYMIO_EVENT_BUTTON_HELPURL);
		this.setTooltip(Blockly.Msg.THYMIO_EVENT_BUTTON_TOOLTIP);
		this.appendStatementInput('HANDLER');
	}
};

Blockly.Blocks['thymio_event_prox'] = {
	/**
	 * Block for Thymio proximity event.
	 * 
	 * @this Blockly.Block
	 */
	init : function()
	{
		this.jsonInit({
			"message0" : Blockly.Msg.THYMIO_EVENT_PROX,
			"args0" : [{
				"type" : "field_dropdown",
				"name" : "SENSOR",
				"options" : [
					[Blockly.Msg.THYMIO_EVENT_PROX_FRONT_LEFT, 'prox.horizontal[0]'],
					[Blockly.Msg.THYMIO_EVENT_PROX_FRONT_LEFT_MIDDLE, 'prox.horizontal[1]'],
					[Blockly.Msg.THYMIO_EVENT_PROX_FRONT_MIDDLE, 'prox.horizontal[2]'],
					[Blockly.Msg.THYMIO_EVENT_PROX_FRONT_RIGHT_MIDDLE, 'prox.horizontal[3]'],
					[Blockly.Msg.THYMIO_EVENT_PROX_FRONT_RIGHT, 'prox.horizontal[4]'],
					[Blockly.Msg.THYMIO_EVENT_PROX_REAR_LEFT, 'prox.horizontal[5]'],
					[Blockly.Msg.THYMIO_EVENT_PROX_REAR_RIGHT, 'prox.horizontal[6]']]
			}, {
				"type" : "field_dropdown",
				"name" : "MODE",
				"options" : [
					[Blockly.Msg.THYMIO_EVENT_PROX_PROX, 'PROX'],
					[Blockly.Msg.THYMIO_EVENT_PROX_NOPROX, 'NOPROX']]
			}]
		});
		
		this.setColour(Blockly.Blocks.thymio.EVENTS_HUE);
		this.setHelpUrl(Blockly.Msg.THYMIO_EVENT_PROX_HELPURL);
		this.setTooltip(Blockly.Msg.THYMIO_EVENT_PROX_TOOLTIP);
		this.appendStatementInput('HANDLER');
	}
};

Blockly.Blocks['thymio_event_prox_ground'] = {
	/**
	 * Block for Thymio ground proximity event.
	 * 
	 * @this Blockly.Block
	 */
	init : function()
	{
		this.jsonInit({
			"message0" : Blockly.Msg.THYMIO_EVENT_PROX_GROUND,
			"args0" : [{
				"type" : "field_dropdown",
				"name" : "SENSOR",
				"options" : [
					[Blockly.Msg.THYMIO_EVENT_PROX_GROUND_LEFT, 'prox.ground.delta[0]'],
					[Blockly.Msg.THYMIO_EVENT_PROX_GROUND_RIGHT, 'prox.ground.delta[1]']]
			}, {
				"type" : "field_dropdown",
				"name" : "MODE",
				"options" : [
					[Blockly.Msg.THYMIO_EVENT_PROX_GROUND_BLACK, 'BLACK'],
					[Blockly.Msg.THYMIO_EVENT_PROX_GROUND_WHITE, 'WHITE'],
					[Blockly.Msg.THYMIO_EVENT_PROX_GROUND_PROX, 'PROX'],
					[Blockly.Msg.THYMIO_EVENT_PROX_GROUND_NOPROX, 'NOPROX']]
			}]
		});
		
		this.setColour(Blockly.Blocks.thymio.EVENTS_HUE);
		this.setHelpUrl(Blockly.Msg.THYMIO_EVENT_PROX_GROUND_HELPURL);
		this.setTooltip(Blockly.Msg.THYMIO_EVENT_PROX_GROUND_TOOLTIP);
		this.appendStatementInput('HANDLER');
	}
};

Blockly.Blocks['thymio_event_shock'] = {
	/**
	 * Block for Thymio shock events.
	 * 
	 * @this Blockly.Block
	 */
	init : function()
	{
		this.setColour(Blockly.Blocks.thymio.EVENTS_HUE);
		this.setHelpUrl(Blockly.Msg.THYMIO_EVENT_SHOCK_HELPURL);
		this.setTooltip(Blockly.Msg.THYMIO_EVENT_SHOCK_TOOLTIP);
		
		this.appendDummyInput().appendField(Blockly.Msg.THYMIO_EVENT_SHOCK);
		this.appendStatementInput('HANDLER');
	}
};

Blockly.Blocks['thymio_event_timer'] = {
	/**
	 * Block for Thymio events.
	 * 
	 * @this Blockly.Block
	 */
	init : function()
	{
		this.jsonInit({
			"message0" : Blockly.Msg.THYMIO_EVENT_TIMER,
			"args0" : [{
				"type" : "field_dropdown",
				"name" : "EVENT",
				"options" : [
					[Blockly.Msg.THYMIO_EVENT_TIMER_FIRST, 'timer0'],
					[Blockly.Msg.THYMIO_EVENT_TIMER_SECOND, 'timer1']]
			}]
		});
		
		this.setColour(Blockly.Blocks.thymio.EVENTS_HUE);
		this.setHelpUrl(Blockly.Msg.THYMIO_EVENT_TIMER_HELPURL);
		this.setTooltip(Blockly.Msg.THYMIO_EVENT_TIMER_TOOLTIP);
		this.appendStatementInput('HANDLER');
	}
};

Blockly.Blocks['thymio_event_sound'] = {
	/**
	 * Block for Thymio sound events.
	 * 
	 * @this Blockly.Block
	 */
	init : function()
	{
		this.jsonInit({
			"message0" : Blockly.Msg.THYMIO_EVENT_SOUND,
			"args0" : [{
				"type" : "field_dropdown",
				"name" : "EVENT",
				"options" : [
					[Blockly.Msg.THYMIO_EVENT_SOUND_MIC, 'mic'],
					[Blockly.Msg.THYMIO_EVENT_SOUND_FINISHED, 'sound.finished']]
			}]
		});
		
		this.setColour(Blockly.Blocks.thymio.EVENTS_HUE);
		this.setHelpUrl(Blockly.Msg.THYMIO_EVENT_SOUND_HELPURL);
		this.setTooltip(Blockly.Msg.THYMIO_EVENT_SOUND_TOOLTIP);
		this.appendStatementInput('HANDLER');
	}
};

Blockly.Blocks['thymio_event_receive'] = {
	/**
	 * Block for Thymio signal events.
	 * 
	 * @this Blockly.Block
	 */
	init : function()
	{
		this.jsonInit({
			"message0" : Blockly.Msg.THYMIO_EVENT_RECEIVE,
			"args0" : [{
				"type" : "field_dropdown",
				"name" : "EVENT",
				"options" : [
					[Blockly.Msg.THYMIO_EVENT_RECEIVE_COMM, 'prox.comm'],
					[Blockly.Msg.THYMIO_EVENT_RECEIVE_RC, 'rc5']]
			}]
		});
		
		this.setColour(Blockly.Blocks.thymio.EVENTS_HUE);
		this.setHelpUrl(Blockly.Msg.THYMIO_EVENT_RECEIVE_HELPURL);
		this.setTooltip(Blockly.Msg.THYMIO_EVENT_RECEIVE_TOOLTIP);
		this.appendStatementInput('HANDLER');
	}
};

Blockly.Blocks['thymio_event_update'] = {
	/**
	 * Block for Thymio update events.
	 * 
	 * @this Blockly.Block
	 */
	init : function()
	{
		this.jsonInit({
			"message0" : Blockly.Msg.THYMIO_EVENT_UPDATE,
			"args0" : [{
				"type" : "field_dropdown",
				"name" : "EVENT",
				"options" : [
					[Blockly.Msg.THYMIO_EVENT_UPDATE_BUTTONS, 'buttons'],
					[Blockly.Msg.THYMIO_EVENT_UPDATE_PROX, 'prox'],
					[Blockly.Msg.THYMIO_EVENT_UPDATE_TEMPERATURE, 'temperature'],
					[Blockly.Msg.THYMIO_EVENT_UPDATE_ACC, 'acc'],
					[Blockly.Msg.THYMIO_EVENT_UPDATE_MOTOR, 'motor']]
			}]
		});
		
		this.setColour(Blockly.Blocks.thymio.EVENTS_HUE);
		this.setHelpUrl(Blockly.Msg.THYMIO_EVENT_UPDATED_HELPURL);
		this.setTooltip(Blockly.Msg.THYMIO_EVENT_UPDATED_TOOLTIP);
		this.appendStatementInput('HANDLER');
	}
};

Blockly.Blocks['thymio_led'] = {
	/**
	 * Block to set Thymio LEDs.
	 * 
	 * @this Blockly.Block
	 */
	init : function()
	{
		this.jsonInit({
			"message0" : Blockly.Msg.THYMIO_LED,
			"args0" : [{
				"type" : "field_dropdown",
				"name" : "LED",
				"options" : [
					[Blockly.Msg.THYMIO_LED_TOP, 'leds.top'],
					[Blockly.Msg.THYMIO_LED_BOTTOM_LEFT, 'leds.bottom.left'],
					[Blockly.Msg.THYMIO_LED_BOTTOM_RIGHT, 'leds.bottom.right']]
			}, {
				"type" : "field_colour",
				"name" : "COLOR",
				"colour" : "#ff0000"
			}]
		});
		
		this.setColour(Blockly.Blocks.thymio.LEDS_HUE);
		this.setHelpUrl(Blockly.Msg.THYMIO_LED_HELPURL);
		this.setTooltip(Blockly.Msg.THYMIO_LED_TOOLTIP);
		this.setPreviousStatement(true);
		this.setNextStatement(true);
	}
};

Blockly.Blocks['thymio_led_rgb'] = {
	/**
	 * Block to set Thymio LEDs by RGB values.
	 * 
	 * @this Blockly.Block
	 */
	init : function()
	{
		this.jsonInit({
			"message0" : Blockly.Msg.THYMIO_LED_RGB,
			"args0" : [{
				"type" : "field_dropdown",
				"name" : "LED",
				"options" : [
					[Blockly.Msg.THYMIO_LED_RGB_TOP, 'leds.top'],
					[Blockly.Msg.THYMIO_LED_RGB_BOTTOM_LEFT, 'leds.bottom.left'],
					[Blockly.Msg.THYMIO_LED_RGB_BOTTOM_RIGHT, 'leds.bottom.right']]
			}, {
				"type" : "input_value",
				"name" : "RED",
				"check" : "Number",
				"align" : "RIGHT"
			}, {
				"type" : "input_value",
				"name" : "GREEN",
				"check" : "Number",
				"align" : "RIGHT"
			}, {
				"type" : "input_value",
				"name" : "BLUE",
				"check" : "Number",
				"align" : "RIGHT"
			}]
		});
		
		this.setColour(Blockly.Blocks.thymio.LEDS_HUE);
		this.setHelpUrl(Blockly.Msg.THYMIO_LED_RGB_HELPURL);
		this.setTooltip(Blockly.Msg.THYMIO_LED_RGB_TOOLTIP);
		this.setPreviousStatement(true);
		this.setNextStatement(true);
	}
};

Blockly.Blocks['thymio_led_circle'] = {
	/**
	 * Block to set Thymio circle leds
	 * 
	 * @this Blockly.Block
	 */
	init : function()
	{
		this.jsonInit({
			"message0" : Blockly.Msg.THYMIO_LED_CIRCLE,
			"args0" : [{
				"type" : "input_value",
				"name" : "CIRCLE0",
				"check" : "Number",
				"align" : "RIGHT"
			}, {
				"type" : "input_value",
				"name" : "CIRCLE1",
				"check" : "Number",
				"align" : "RIGHT"
			}, {
				"type" : "input_value",
				"name" : "CIRCLE2",
				"check" : "Number",
				"align" : "RIGHT"
			}, {
				"type" : "input_value",
				"name" : "CIRCLE3",
				"check" : "Number",
				"align" : "RIGHT"
			}, {
				"type" : "input_value",
				"name" : "CIRCLE4",
				"check" : "Number",
				"align" : "RIGHT"
			}, {
				"type" : "input_value",
				"name" : "CIRCLE5",
				"check" : "Number",
				"align" : "RIGHT"
			}, {
				"type" : "input_value",
				"name" : "CIRCLE6",
				"check" : "Number",
				"align" : "RIGHT"
			}, {
				"type" : "input_value",
				"name" : "CIRCLE7",
				"check" : "Number",
				"align" : "RIGHT"
			}]
		});
		
		this.setColour(Blockly.Blocks.thymio.LEDS_HUE);
		this.setHelpUrl(Blockly.Msg.THYMIO_LED_CIRCLE_HELPURL);
		this.setTooltip(Blockly.Msg.THYMIO_LED_CIRCLE_TOOLTIP);
		this.setPreviousStatement(true);
		this.setNextStatement(true);
	}
};

Blockly.Blocks['thymio_led_prox'] = {
	/**
	 * Block to set Thymio proximity leds
	 * 
	 * @this Blockly.Block
	 */
	init : function()
	{
		this.jsonInit({
			"message0" : Blockly.Msg.THYMIO_LED_PROX,
			"args0" : [{
				"type" : "input_value",
				"name" : "PROX0",
				"check" : "Number",
				"align" : "RIGHT"
			}, {
				"type" : "input_value",
				"name" : "PROX1",
				"check" : "Number",
				"align" : "RIGHT"
			}, {
				"type" : "input_value",
				"name" : "PROX2",
				"check" : "Number",
				"align" : "RIGHT"
			}, {
				"type" : "input_value",
				"name" : "PROX3",
				"check" : "Number",
				"align" : "RIGHT"
			}, {
				"type" : "input_value",
				"name" : "PROX4",
				"check" : "Number",
				"align" : "RIGHT"
			}, {
				"type" : "input_value",
				"name" : "PROX5",
				"check" : "Number",
				"align" : "RIGHT"
			}, {
				"type" : "input_value",
				"name" : "PROX6",
				"check" : "Number",
				"align" : "RIGHT"
			}, {
				"type" : "input_value",
				"name" : "PROX7",
				"check" : "Number",
				"align" : "RIGHT"
			}]
		});
		
		this.setColour(Blockly.Blocks.thymio.LEDS_HUE);
		this.setHelpUrl(Blockly.Msg.THYMIO_LED_PROX_HELPURL);
		this.setTooltip(Blockly.Msg.THYMIO_LED_PROX_TOOLTIP);
		this.setPreviousStatement(true);
		this.setNextStatement(true);
	}
};


Blockly.Blocks['thymio_led_prox_ground'] = {
	/**
	 * Block to set Thymio proximity leds
	 * 
	 * @this Blockly.Block
	 */
	init : function()
	{
		this.jsonInit({
			"message0" : Blockly.Msg.THYMIO_LED_PROX_GROUND,
			"args0" : [{
				"type" : "input_value",
				"name" : "PROX0",
				"check" : "Number",
				"align" : "RIGHT"
			}, {
				"type" : "input_value",
				"name" : "PROX1",
				"check" : "Number",
				"align" : "RIGHT"
			}]
		});
		
		this.setColour(Blockly.Blocks.thymio.LEDS_HUE);
		this.setHelpUrl(Blockly.Msg.THYMIO_LED_PROX_GROUND_HELPURL);
		this.setTooltip(Blockly.Msg.THYMIO_LED_PROX_GROUND_TOOLTIP);
		this.setPreviousStatement(true);
		this.setNextStatement(true);
	}
};

Blockly.Blocks['thymio_led_button'] = {
	/**
	 * Block to set Thymio button leds
	 * 
	 * @this Blockly.Block
	 */
	init : function()
	{
		this.jsonInit({
			"message0" : Blockly.Msg.THYMIO_LED_BUTTON,
			"args0" : [{
				"type" : "input_value",
				"name" : "FORWARD",
				"check" : "Number",
				"align" : "RIGHT"
			}, {
				"type" : "input_value",
				"name" : "RIGHT",
				"check" : "Number",
				"align" : "RIGHT"
			}, {
				"type" : "input_value",
				"name" : "BACKWARD",
				"check" : "Number",
				"align" : "RIGHT"
			}, {
				"type" : "input_value",
				"name" : "LEFT",
				"check" : "Number",
				"align" : "RIGHT"
			}]
		});
		
		this.setColour(Blockly.Blocks.thymio.LEDS_HUE);
		this.setHelpUrl(Blockly.Msg.THYMIO_LED_BUTTON_HELPURL);
		this.setTooltip(Blockly.Msg.THYMIO_LED_BUTTON_TOOLTIP);
		this.setPreviousStatement(true);
		this.setNextStatement(true);
	}
};

Blockly.Blocks['thymio_led_temperature'] = {
	/**
	 * Block to set Thymio temperature leds
	 * 
	 * @this Blockly.Block
	 */
	init : function()
	{
		this.jsonInit({
			"message0" : Blockly.Msg.THYMIO_LED_TEMPERATURE,
			"args0" : [{
				"type" : "input_value",
				"name" : "RED",
				"check" : "Number",
				"align" : "RIGHT"
			}, {
				"type" : "input_value",
				"name" : "BLUE",
				"check" : "Number",
				"align" : "RIGHT"
			}]
		});
		
		this.setColour(Blockly.Blocks.thymio.LEDS_HUE);
		this.setHelpUrl(Blockly.Msg.THYMIO_LED_TEMPERATURE_HELPURL);
		this.setTooltip(Blockly.Msg.THYMIO_LED_TEMPERATURE_TOOLTIP);
		this.setPreviousStatement(true);
		this.setNextStatement(true);
	}
};

Blockly.Blocks['thymio_led_rc_sound'] = {
	/**
	 * Block to set Thymio rc and sound leds
	 * 
	 * @this Blockly.Block
	 */
	init : function()
	{
		this.jsonInit({
			"message0" : Blockly.Msg.THYMIO_LED_RC_SOUND,
			"args0" : [{
				"type" : "field_dropdown",
				"name" : "LED",
				"options" : [
					[Blockly.Msg.THYMIO_LED_RC_SOUND_RC, 'leds.rc'],
					[Blockly.Msg.THYMIO_LED_RC_SOUND_SOUND, 'leds.sound']]
			}, {
				"type" : "input_value",
				"name" : "INTENSITY",
				"check" : "Number",
				"align" : "RIGHT"
			}]
		});
		
		this.setColour(Blockly.Blocks.thymio.LEDS_HUE);
		this.setHelpUrl(Blockly.Msg.THYMIO_LED_RC_SOUND_HELPURL);
		this.setTooltip(Blockly.Msg.THYMIO_LED_RC_SOUND_TOOLTIP);
		this.setPreviousStatement(true);
		this.setNextStatement(true);
	}
};

Blockly.Blocks['thymio_led_off'] = {
	/**
	 * Block to turn off Thymio LEDs.
	 * 
	 * @this Blockly.Block
	 */
	init : function()
	{
		this.jsonInit({
			"message0" : Blockly.Msg.THYMIO_LED_OFF,
			"args0" : [{
				"type" : "field_dropdown",
				"name" : "LED",
				"options" : [
					[Blockly.Msg.THYMIO_LED_OFF_TOP, 'leds.top'],
					[Blockly.Msg.THYMIO_LED_OFF_BOTTOM_LEFT, 'leds.bottom.left'],
					[Blockly.Msg.THYMIO_LED_OFF_BOTTOM_RIGHT, 'leds.bottom.right'],
					[Blockly.Msg.THYMIO_LED_OFF_CIRCLE, 'leds.circle'],
					[Blockly.Msg.THYMIO_LED_OFF_PROX_H, 'leds.prox.h'],
					[Blockly.Msg.THYMIO_LED_OFF_PROX_V, 'leds.prox.v'],
					[Blockly.Msg.THYMIO_LED_OFF_RC, 'leds.rc'],
					[Blockly.Msg.THYMIO_LED_OFF_BUTTONS, 'leds.buttons'],
					[Blockly.Msg.THYMIO_LED_OFF_TEMPERATURE, 'leds.temperature'],
					[Blockly.Msg.THYMIO_LED_OFF_MICROPHONE, 'leds.sound']]
			}]
		});
		
		this.setColour(Blockly.Blocks.thymio.LEDS_HUE);
		this.setHelpUrl(Blockly.Msg.THYMIO_LED_OFF_HELPURL);
		this.setTooltip(Blockly.Msg.THYMIO_LED_OFF_TOOLTIP);
		this.setPreviousStatement(true);
		this.setNextStatement(true);
	}
};

Blockly.Blocks['thymio_sound_system'] = {
	/**
	 * Block to play Thymio system sounds.
	 * 
	 * @this Blockly.Block
	 */
	init : function()
	{
		this.jsonInit({
			"message0" : Blockly.Msg.THYMIO_SOUND_SYSTEM,
			"args0" : [{
				"type" : "field_dropdown",
				"name" : "SOUND",
				"options" : [
					[Blockly.Msg.THYMIO_SOUND_SYSTEM_STARTUP, '0'],
					[Blockly.Msg.THYMIO_SOUND_SYSTEM_SHUTDOWN, '1'],
					[Blockly.Msg.THYMIO_SOUND_SYSTEM_ARROW, '2'],
					[Blockly.Msg.THYMIO_SOUND_SYSTEM_CENTRAL, '3'],
					[Blockly.Msg.THYMIO_SOUND_SYSTEM_SCARY, '4'],
					[Blockly.Msg.THYMIO_SOUND_SYSTEM_COLLISION, '5'],
					[Blockly.Msg.THYMIO_SOUND_SYSTEM_TARGET_FRIENDLY, '6'],
					[Blockly.Msg.THYMIO_SOUND_SYSTEM_TARGET_DETECTED, '7']]
			}]
		});
		
		this.setColour(Blockly.Blocks.thymio.ACTUATORS_HUE);
		this.setHelpUrl(Blockly.Msg.THYMIO_SOUND_SYSTEM_HELPURL);
		this.setTooltip(Blockly.Msg.THYMIO_SOUND_SYSTEM_TOOLTIP);
		this.setPreviousStatement(true);
		this.setNextStatement(true);
	}
};

Blockly.Blocks['thymio_sound_note'] = {
	/**
	 * Block to play Thymio sound notes.
	 * 
	 * @this Blockly.Block
	 */
	init : function()
	{
		this.jsonInit({
			"message0" : Blockly.Msg.THYMIO_SOUND_NOTE,
			"args0" : [{
				"type" : "input_value",
				"name" : "FREQ",
				"check" : "Number"
			}, {
				"type" : "input_value",
				"name" : "DURATION",
				"check" : "Number"
			}]
		});
		
		this.setColour(Blockly.Blocks.thymio.ACTUATORS_HUE);
		this.setHelpUrl(Blockly.Msg.THYMIO_SOUND_NOTE_HELPURL);
		this.setTooltip(Blockly.Msg.THYMIO_SOUND_NOTE_TOOLTIP);
		this.setPreviousStatement(true);
		this.setNextStatement(true);
	}
};

Blockly.Blocks['thymio_sound_stop'] = {
	/**
	 * Block to cause Thymio to stop playing sound.
	 * 
	 * @this Blockly.Block
	 */
	init : function()
	{
		this.setColour(Blockly.Blocks.thymio.ACTUATORS_HUE);
		this.setHelpUrl(Blockly.Msg.THYMIO_SOUND_STOP_HELPURL);
		this.setTooltip(Blockly.Msg.THYMIO_SOUND_STOP_TOOLTIP);
		this.setPreviousStatement(true);
		this.setNextStatement(true);
		this.appendDummyInput().appendField(Blockly.Msg.THYMIO_SOUND_STOP);
	}
};

Blockly.Blocks['thymio_button_pressed'] = {
	/**
	 * Block for checking whether a button is pressed
	 * 
	 * @this Blockly.Block
	 */
	init : function()
	{
		this.jsonInit({
			"message0" : Blockly.Msg.THYMIO_BUTTON_PRESSED,
			"args0" : [{
				"type" : "field_dropdown",
				"name" : "BUTTON",
				"options" : [
					[Blockly.Msg.THYMIO_BUTTON_PRESSED_CENTER, 'button.center'],
					[Blockly.Msg.THYMIO_BUTTON_PRESSED_FORWARD, 'button.forward'],
					[Blockly.Msg.THYMIO_BUTTON_PRESSED_BACKWARD, 'button.backward'],
					[Blockly.Msg.THYMIO_BUTTON_PRESSED_LEFT, 'button.left'],
					[Blockly.Msg.THYMIO_BUTTON_PRESSED_RIGHT, 'button.right']]
			}]
		});
		
		this.setColour(Blockly.Blocks.logic.HUE);
		this.setHelpUrl(Blockly.Msg.THYMIO_BUTTON_PRESSED_HELPURL);
		this.setTooltip(Blockly.Msg.THYMIO_BUTTON_PRESSED_TOOLTIP);
		this.setOutput(true, 'Boolean');
	}
};

Blockly.Blocks['thymio_prox_check'] = {
	/**
	 * Block for checking whether a proximity sensor is blocked or cleared
	 * 
	 * @this Blockly.Block
	 */
	init : function()
	{
		this.jsonInit({
			"message0" : Blockly.Msg.THYMIO_PROX_CHECK,
			"args0" : [{
				"type" : "field_dropdown",
				"name" : "SENSOR",
				"options" : [
					[Blockly.Msg.THYMIO_PROX_CHECK_FRONT_LEFT, 'prox.horizontal[0]'],
					[Blockly.Msg.THYMIO_PROX_CHECK_FRONT_LEFT_MIDDLE, 'prox.horizontal[1]'],
					[Blockly.Msg.THYMIO_PROX_CHECK_FRONT_MIDDLE, 'prox.horizontal[2]'],
					[Blockly.Msg.THYMIO_PROX_CHECK_FRONT_RIGHT_MIDDLE, 'prox.horizontal[3]'],
					[Blockly.Msg.THYMIO_PROX_CHECK_FRONT_RIGHT, 'prox.horizontal[4]'],
					[Blockly.Msg.THYMIO_PROX_CHECK_REAR_LEFT, 'prox.horizontal[5]'],
					[Blockly.Msg.THYMIO_PROX_CHECK_REAR_RIGHT, 'prox.horizontal[6]']]
			}, {
				"type" : "field_dropdown",
				"name" : "MODE",
				"options" : [
					[Blockly.Msg.THYMIO_PROX_CHECK_PROX, 'PROX'],
					[Blockly.Msg.THYMIO_PROX_CHECK_NOPROX, 'NOPROX']]
			}]
		});
		
		this.setColour(Blockly.Blocks.logic.HUE);
		this.setHelpUrl(Blockly.Msg.THYMIO_PROX_CHECK_HELPURL);
		this.setTooltip(Blockly.Msg.THYMIO_PROX_CHECK_TOOLTIP);
		this.setOutput(true, 'Boolean');
	}
};

Blockly.Blocks['thymio_prox_ground_check'] = {
	/**
	 * Block for checking whether a ground proximity sensor is white or black
	 * 
	 * @this Blockly.Block
	 */
	init : function()
	{
		this.jsonInit({
			"message0" : Blockly.Msg.THYMIO_PROX_GROUND_CHECK,
			"args0" : [{
				"type" : "field_dropdown",
				"name" : "SENSOR",
				"options" : [
					[Blockly.Msg.THYMIO_PROX_GROUND_CHECK_LEFT, 'prox.ground.delta[0]'],
					[Blockly.Msg.THYMIO_PROX_GROUND_CHECK_RIGHT, 'prox.ground.delta[1]']]
			}, {
				"type" : "field_dropdown",
				"name" : "MODE",
				"options" : [
					[Blockly.Msg.THYMIO_PROX_GROUND_CHECK_BLACK, 'BLACK'],
					[Blockly.Msg.THYMIO_PROX_GROUND_CHECK_WHITE, 'WHITE'],
					[Blockly.Msg.THYMIO_PROX_GROUND_CHECK_PROX, 'PROX'],
					[Blockly.Msg.THYMIO_PROX_GROUND_CHECK_NOPROX, 'NOPROX']]
			}]
		});
		
		this.setColour(Blockly.Blocks.logic.HUE);
		this.setHelpUrl(Blockly.Msg.THYMIO_PROX_GROUND_CHECK_HELPURL);
		this.setTooltip(Blockly.Msg.THYMIO_PROX_GROUND_CHECK_TOOLTIP);
		this.setOutput(true, 'Boolean');
	}
};

Blockly.Blocks['thymio_sensor_temperature'] = {
	/**
	 * Block for retrieving the temperature sensor state
	 * 
	 * @this Blockly.Block
	 */
	init : function()
	{
		this.setColour(Blockly.Blocks.math.HUE);
		this.setHelpUrl(Blockly.Msg.THYMIO_SENSOR_TEMPERATURE_HELPURL);
		this.setTooltip(Blockly.Msg.THYMIO_SENSOR_TEMPERATURE_TOOLTIP);

		this.setOutput(true, 'Number');
		this.appendDummyInput().appendField(Blockly.Msg.THYMIO_SENSOR_TEMPERATURE);
	}
};

Blockly.Blocks['thymio_sensor_mic'] = {
	/**
	 * Block for retrieving the microphone intensity sensor state
	 * 
	 * @this Blockly.Block
	 */
	init : function()
	{
		this.setColour(Blockly.Blocks.math.HUE);
		this.setHelpUrl(Blockly.Msg.THYMIO_SENSOR_MIC_HELPURL);
		this.setTooltip(Blockly.Msg.THYMIO_SENSOR_MIC_TOOLTIP);

		this.setOutput(true, 'Number');
		this.appendDummyInput().appendField(Blockly.Msg.THYMIO_SENSOR_MIC);
	}
};

Blockly.Blocks['thymio_sensor_comm'] = {
	/**
	 * Block for retrieving the IR communication value
	 * 
	 * @this Blockly.Block
	 */
	init : function()
	{
		this.setColour(Blockly.Blocks.math.HUE);
		this.setHelpUrl(Blockly.Msg.THYMIO_SENSOR_COMM_HELPURL);
		this.setTooltip(Blockly.Msg.THYMIO_SENSOR_COMM_TOOLTIP);

		this.setOutput(true, 'Number');
		this.appendDummyInput().appendField(Blockly.Msg.THYMIO_SENSOR_COMM);
	}
};

Blockly.Blocks['thymio_sensor_prox'] = {
	/**
	 * Block for retrieving a proximity sensor state
	 * 
	 * @this Blockly.Block
	 */
	init : function()
	{
		this.jsonInit({
			"message0" : Blockly.Msg.THYMIO_SENSOR_PROX,
			"args0" : [{
				"type" : "field_dropdown",
				"name" : "SENSOR",
				"options" : [
					[Blockly.Msg.THYMIO_SENSOR_PROX_FRONT_LEFT, 'prox.horizontal[0]'],
					[Blockly.Msg.THYMIO_SENSOR_PROX_FRONT_LEFT_MIDDLE, 'prox.horizontal[1]'],
					[Blockly.Msg.THYMIO_SENSOR_PROX_FRONT_MIDDLE, 'prox.horizontal[2]'],
					[Blockly.Msg.THYMIO_SENSOR_PROX_FRONT_RIGHT_MIDDLE, 'prox.horizontal[3]'],
					[Blockly.Msg.THYMIO_SENSOR_PROX_FRONT_RIGHT, 'prox.horizontal[4]'],
					[Blockly.Msg.THYMIO_SENSOR_PROX_REAR_LEFT, 'prox.horizontal[5]'],
					[Blockly.Msg.THYMIO_SENSOR_PROX_REAR_RIGHT, 'prox.horizontal[6]'],
					[Blockly.Msg.THYMIO_SENSOR_PROX_GROUND_LEFT, 'prox.ground.delta[0]'],
					[Blockly.Msg.THYMIO_SENSOR_PROX_GROUND_RIGHT, 'prox.ground.delta[1]']]
			}]
		});
		
		this.setColour(Blockly.Blocks.math.HUE);
		this.setHelpUrl(Blockly.Msg.THYMIO_SENSOR_PROX_HELPURL);
		this.setTooltip(Blockly.Msg.THYMIO_SENSOR_PROX_TOOLTIP);
		this.setOutput(true, 'Number');
	}
};

Blockly.Blocks['thymio_sensor_motor'] = {
	/**
	 * Block for retrieving a motor sensor state
	 * 
	 * @this Blockly.Block
	 */
	init : function()
	{
		this.jsonInit({
			"message0" : Blockly.Msg.THYMIO_SENSOR_MOTOR,
			"args0" : [{
				"type" : "field_dropdown",
				"name" : "SENSOR",
				"options" : [
					[Blockly.Msg.THYMIO_SENSOR_MOTOR_LEFT, 'motor.left.speed'],
					[Blockly.Msg.THYMIO_SENSOR_MOTOR_RIGHT, 'motor.right.speed']]
			}]
		});
		
		this.setColour(Blockly.Blocks.math.HUE);
		this.setHelpUrl(Blockly.Msg.THYMIO_SENSOR_MOTOR_HELPURL);
		this.setTooltip(Blockly.Msg.THYMIO_SENSOR_MOTOR_TOOLTIP);
		this.setOutput(true, 'Number');
	}
};

Blockly.Blocks['thymio_sensor_acc'] = {
	/**
	 * Block for retrieving a accelerometer sensor state
	 * 
	 * @this Blockly.Block
	 */
	init : function()
	{
		this.jsonInit({
			"message0" : Blockly.Msg.THYMIO_SENSOR_ACC,
			"args0" : [{
				"type" : "field_dropdown",
				"name" : "SENSOR",
				"options" : [
					[Blockly.Msg.THYMIO_SENSOR_ACC_X, 'acc[0]'],
					[Blockly.Msg.THYMIO_SENSOR_ACC_Y, 'acc[1]'],
					[Blockly.Msg.THYMIO_SENSOR_ACC_Z, 'acc[2]']]
			}]
		});
		
		this.setColour(Blockly.Blocks.math.HUE);
		this.setHelpUrl(Blockly.Msg.THYMIO_SENSOR_ACC_HELPURL);
		this.setTooltip(Blockly.Msg.THYMIO_SENSOR_ACC_TOOLTIP);
		this.setOutput(true, 'Number');
	}
};

Blockly.Blocks['thymio_sensor_rc'] = {
	/**
	 * Block for retrieving a remote control sensor state
	 * 
	 * @this Blockly.Block
	 */
	init : function()
	{
		this.jsonInit({
			"message0" : Blockly.Msg.THYMIO_SENSOR_RC,
			"args0" : [{
				"type" : "field_dropdown",
				"name" : "SENSOR",
				"options" : [
					[Blockly.Msg.THYMIO_SENSOR_RC_ADDRESS, 'rc5.address'],
					[Blockly.Msg.THYMIO_SENSOR_RC_COMMAND, 'rc5.command']]
			}]
		});
		
		this.setColour(Blockly.Blocks.math.HUE);
		this.setHelpUrl(Blockly.Msg.THYMIO_SENSOR_RC_HELPURL);
		this.setTooltip(Blockly.Msg.THYMIO_SENSOR_RC_TOOLTIP);
		this.setOutput(true, 'Number');
	}
};

Blockly.Blocks['thymio_motors_start'] = {
	/**
	 * Block for starting Thymio's motors
	 * 
	 * @this Blockly.Block
	 */
	init : function()
	{
		this.jsonInit({
			"message0" : Blockly.Msg.THYMIO_MOTORS_START,
			"args0" : [{
				"type" : "field_dropdown",
				"name" : "COMMAND",
				"options" : [
					[Blockly.Msg.THYMIO_MOTORS_START_FORWARD, 'FORWARD'],
					[Blockly.Msg.THYMIO_MOTORS_START_BACKWARD, 'BACKWARD'],
					[Blockly.Msg.THYMIO_MOTORS_START_TURNLEFT, 'TURNLEFT'],
					[Blockly.Msg.THYMIO_MOTORS_START_TURNRIGHT, 'TURNRIGHT'],
					[Blockly.Msg.THYMIO_MOTORS_START_TURNBACKWARDLEFT, 'TURNBACKWARDLEFT'],
					[Blockly.Msg.THYMIO_MOTORS_START_TURNBACKWARDRIGHT, 'TURNBACKWARDRIGHT'],
					[Blockly.Msg.THYMIO_MOTORS_START_SPINCCW, 'SPINCCW'],
					[Blockly.Msg.THYMIO_MOTORS_START_SPINCW, 'SPINCW']]
			}, {
				"type" : "input_value",
				"name" : "SPEED",
				"check" : "Number"
			}]
		});
		
		this.setColour(Blockly.Blocks.thymio.ACTUATORS_HUE);
		this.setHelpUrl(Blockly.Msg.THYMIO_MOTORS_START_HELPURL);
		this.setTooltip(Blockly.Msg.THYMIO_MOTORS_START_TOOLTIP);
		this.setPreviousStatement(true);
		this.setNextStatement(true);
	}
};

Blockly.Blocks['thymio_motors_stop'] = {
	/**
	 * Block for stopping Thymio's motors
	 * 
	 * @this Blockly.Block
	 */
	init : function()
	{
		this.setColour(Blockly.Blocks.thymio.ACTUATORS_HUE);
		this.setHelpUrl(Blockly.Msg.THYMIO_MOTORS_STOP_HELPURL);
		this.setTooltip(Blockly.Msg.THYMIO_MOTORS_STOP_TOOLTIP);
		this.setPreviousStatement(true);
		this.setNextStatement(true);
		
		this.appendDummyInput().appendField(Blockly.Msg.THYMIO_MOTORS_STOP);
	}
};

Blockly.Blocks['thymio_actuator_mic'] = {
	/**
	 * Block for setting a Thymio microphone threshold
	 * 
	 * @this Blockly.Block
	 */
	init : function()
	{
		this.setColour(Blockly.Blocks.thymio.ACTUATORS_HUE);
		this.setHelpUrl(Blockly.Msg.THYMIO_ACTUATOR_MIC_HELPURL);
		this.setTooltip(Blockly.Msg.THYMIO_ACTUATOR_MIC_TOOLTIP);
		this.setPreviousStatement(true);
		this.setNextStatement(true);
	
		this.appendValueInput('VALUE').setCheck('Number').appendField(Blockly.Msg.THYMIO_ACTUATOR_MIC);
	}
};

Blockly.Blocks['thymio_actuator_comm'] = {
	/**
	 * Block for setting an IR communication to send
	 * 
	 * @this Blockly.Block
	 */
	init : function()
	{
		this.setColour(Blockly.Blocks.thymio.ACTUATORS_HUE);
		this.setHelpUrl(Blockly.Msg.THYMIO_ACTUATOR_COMM_HELPURL);
		this.setTooltip(Blockly.Msg.THYMIO_ACTUATOR_COMM_TOOLTIP);
		this.setPreviousStatement(true);
		this.setNextStatement(true);
	
		this.appendValueInput('VALUE').setCheck('Number').appendField(Blockly.Msg.THYMIO_ACTUATOR_COMM);
	}
};

Blockly.Blocks['thymio_actuator_timer'] = {
	/**
	 * Block for setting a Thymio timer period
	 * 
	 * @this Blockly.Block
	 */
	init : function()
	{
		this.jsonInit({
			"message0" : Blockly.Msg.THYMIO_ACTUATOR_TIMER,
			"args0" : [{
				"type" : "field_dropdown",
				"name" : "VARIABLE",
				"options" : [
					[Blockly.Msg.THYMIO_ACTUATOR_TIMER_FIRST, 'timer.period[0]'],
					[Blockly.Msg.THYMIO_ACTUATOR_TIMER_SECOND, 'timer.period[1]']]
			}, {
				"type" : "input_value",
				"name" : "VALUE",
				"check" : "Number"
			}]
		});
		
		this.setColour(Blockly.Blocks.thymio.ACTUATORS_HUE);
		this.setHelpUrl(Blockly.Msg.THYMIO_ACTUATOR_TIMER_HELPURL);
		this.setTooltip(Blockly.Msg.THYMIO_ACTUATOR_TIMER_TOOLTIP);
		this.setPreviousStatement(true);
		this.setNextStatement(true);
		this.setInputsInline(true);
	}
};

Blockly.Blocks['thymio_actuator_motor'] = {
	/**
	 * Block for setting a Thymio motor actuator
	 * 
	 * @this Blockly.Block
	 */
	init : function()
	{
		this.jsonInit({
			"message0" : Blockly.Msg.THYMIO_ACTUATOR_MOTOR,
			"args0" : [{
				"type" : "field_dropdown",
				"name" : "VARIABLE",
				"options" : [
					[Blockly.Msg.THYMIO_ACTUATOR_MOTOR_LEFT, 'motor.left.target'],
					[Blockly.Msg.THYMIO_ACTUATOR_MOTOR_RIGHT, 'motor.right.target']]
			}, {
				"type" : "input_value",
				"name" : "VALUE",
				"check" : "Number"
			}]
		});
		
		this.setColour(Blockly.Blocks.thymio.ACTUATORS_HUE);
		this.setHelpUrl(Blockly.Msg.THYMIO_ACTUATOR_MOTOR_HELPURL);
		this.setTooltip(Blockly.Msg.THYMIO_ACTUATOR_MOTOR_TOOLTIP);
		this.setPreviousStatement(true);
		this.setNextStatement(true);
	}
};

Blockly.Blocks['thymio_variable_get'] = {
	/**
	 * Block for variable getter. This is a copy paste of blockly's native variables_get block, except that this one returns only Numbers
	 * 
	 * @this Blockly.Block
	 */
	init : function()
	{
		this.setHelpUrl(Blockly.Msg.VARIABLES_GET_HELPURL);
		this.setColour(Blockly.Blocks.variables.HUE);
		this.appendDummyInput().appendField(new Blockly.FieldVariable(Blockly.Msg.VARIABLES_DEFAULT_NAME), 'VAR');
		this.setOutput(true, 'Number');
		this.setTooltip(Blockly.Msg.VARIABLES_GET_TOOLTIP);
		this.contextMenuMsg_ = Blockly.Msg.VARIABLES_GET_CREATE_SET;
	},
	/**
	 * Return all variables referenced by this block.
	 * 
	 * @return {!Array.<string>} List of variable names.
	 * @this Blockly.Block
	 */
	getVars : function()
	{
		return [this.getFieldValue('VAR')];
	},
	/**
	 * Notification that a variable is renaming. If the name matches one of this
	 * block's variables, rename it.
	 * 
	 * @param {string}
	 *            oldName Previous name of variable.
	 * @param {string}
	 *            newName Renamed variable.
	 * @this Blockly.Block
	 */
	renameVar : function(oldName, newName)
	{
		if(Blockly.Names.equals(oldName, this.getFieldValue('VAR'))) {
			this.setFieldValue(newName, 'VAR');
		}
	},
	contextMenuType_ : 'thymio_variable_set',
	/**
	 * Add menu option to create getter/setter block for this setter/getter.
	 * 
	 * @param {!Array}
	 *            options List of menu options to add to.
	 * @this Blockly.Block
	 */
	customContextMenu : function(options)
	{
		var option = {
			enabled : true
		};
		var name = this.getFieldValue('VAR');
		option.text = this.contextMenuMsg_.replace('%1', name);
		var xmlField = goog.dom.createDom('field', null, name);
		xmlField.setAttribute('name', 'VAR');
		var xmlBlock = goog.dom.createDom('block', null, xmlField);
		xmlBlock.setAttribute('type', this.contextMenuType_);
		option.callback = Blockly.ContextMenu.callbackFactory(this, xmlBlock);
		options.push(option);
	}
};

Blockly.Blocks['thymio_variable_set'] = {
	/**
	 * Block for variable setter. This is a copy paste of blockly's native variables_set block, except that this one accepts only Numbers
	 * 
	 * @this Blockly.Block
	 */
	init : function()
	{
		this.jsonInit({
			"message0" : Blockly.Msg.THYMIO_VARIABLE_SET,
			"args0" : [{
				"type" : "field_variable",
				"name" : "VAR",
				"variable" : Blockly.Msg.VARIABLES_DEFAULT_NAME
			}, {
				"type" : "input_value",
				"name" : "VALUE",
				"check": "Number"
			}],
			"previousStatement" : null,
			"nextStatement" : null,
			"colour" : Blockly.Blocks.variables.HUE,
			"tooltip" : Blockly.Msg.VARIABLES_SET_TOOLTIP,
			"helpUrl" : Blockly.Msg.VARIABLES_SET_HELPURL
		});
		this.contextMenuMsg_ = Blockly.Msg.VARIABLES_SET_CREATE_GET;
	},
	/**
	 * Return all variables referenced by this block.
	 * 
	 * @return {!Array.<string>} List of variable names.
	 * @this Blockly.Block
	 */
	getVars : function()
	{
		return [this.getFieldValue('VAR')];
	},
	/**
	 * Notification that a variable is renaming. If the name matches one of this
	 * block's variables, rename it.
	 * 
	 * @param {string}
	 *            oldName Previous name of variable.
	 * @param {string}
	 *            newName Renamed variable.
	 * @this Blockly.Block
	 */
	renameVar : function(oldName, newName)
	{
		if(Blockly.Names.equals(oldName, this.getFieldValue('VAR'))) {
			this.setFieldValue(newName, 'VAR');
		}
	},
	contextMenuType_ : 'thymio_variable_get',
	customContextMenu : Blockly.Blocks['thymio_variable_get'].customContextMenu
};


Blockly.Blocks['thymio_declare_array'] = {
	/**
	 * Block to declare Thymio arrays.
	 * 
	 * @this Blockly.Block
	 */
	init : function()
	{
		this.jsonInit({
			"message0" : Blockly.Msg.THYMIO_DECLARE_ARRAY,
			"args0" : [{
				"type" : "field_variable",
				"name" : "VAR",
				"variable" : Blockly.Msg.VARIABLES_DEFAULT_NAME
			}, {
				"type" : "field_input",
				"name" : "SIZE",
				"text" : "3"
			}]
		});
		
		this.setColour(Blockly.Blocks.variables.HUE);
		this.setHelpUrl(Blockly.Msg.THYMIO_DECLARE_ARRAY_HELPURL);
		this.setTooltip(Blockly.Msg.THYMIO_DECLARE_ARRAY_TOOLTIP);
		this.getField('SIZE').setChangeHandler(Blockly.FieldTextInput.numberValidator);
	},
	/**
	 * Return all variables referenced by this block.
	 * 
	 * @return {!Array.<string>} List of variable names.
	 * @this Blockly.Block
	 */
	getVars : function()
	{
		return [this.getFieldValue('VAR')];
	},
	/**
	 * Notification that a variable is renaming. If the name matches one of this
	 * block's variables, rename it.
	 * 
	 * @param {string} oldName Previous name of variable.
	 * @param {string} newName Renamed variable.
	 * @this Blockly.Block
	 */
	renameVar : function(oldName, newName)
	{
		if(Blockly.Names.equals(oldName, this.getFieldValue('VAR'))) {
			this.setFieldValue(newName, 'VAR');
		}
	}
};

Blockly.Blocks['thymio_set_array'] = {
	/**
	 * Block for setting a array element
	 * 
	 * @this Blockly.Block
	 */
	init : function()
	{
		this.jsonInit({
			"message0" : Blockly.Msg.THYMIO_SET_ARRAY,
			"args0" : [{
				"type" : "field_variable",
				"name" : "VAR",
				"variable" : Blockly.Msg.VARIABLES_DEFAULT_NAME
			}, {
				"type" : "input_value",
				"name" : "INDEX",
				"check": "Number"
			}, {
				"type" : "input_value",
				"name" : "VALUE",
				"check": "Number"
			}]
		});
		
		this.setColour(Blockly.Blocks.variables.HUE);
		this.setHelpUrl(Blockly.Msg.THYMIO_SET_ARRAY_HELPURL);
		this.setTooltip(Blockly.Msg.THYMIO_SET_ARRAY_TOOLTIP);
		this.setPreviousStatement(true);
		this.setNextStatement(true);
	    this.setInputsInline(true);
	},
	/**
	 * Return all variables referenced by this block.
	 * 
	 * @return {!Array.<string>} List of variable names.
	 * @this Blockly.Block
	 */
	getVars : function()
	{
		return [this.getFieldValue('VAR')];
	},
	/**
	 * Notification that a variable is renaming. If the name matches one of this
	 * block's variables, rename it.
	 * 
	 * @param {string} oldName Previous name of variable.
	 * @param {string} newName Renamed variable.
	 * @this Blockly.Block
	 */
	renameVar : function(oldName, newName)
	{
		if(Blockly.Names.equals(oldName, this.getFieldValue('VAR'))) {
			this.setFieldValue(newName, 'VAR');
		}
	}
};

Blockly.Blocks['thymio_get_array'] = {
	/**
	 * Block for getting a array element
	 * 
	 * @this Blockly.Block
	 */
	init : function()
	{
		this.jsonInit({
			"message0" : Blockly.Msg.THYMIO_GET_ARRAY,
			"args0" : [{
				"type" : "field_variable",
				"name" : "VAR",
				"variable" : Blockly.Msg.VARIABLES_DEFAULT_NAME
			}, {
				"type" : "input_value",
				"name" : "INDEX",
				"check": "Number"
			}]
		});
		
		this.setColour(Blockly.Blocks.variables.HUE);
		this.setHelpUrl(Blockly.Msg.THYMIO_GET_ARRAY_HELPURL);
		this.setTooltip(Blockly.Msg.THYMIO_GET_ARRAY_TOOLTIP);
		this.setOutput(true, 'Number');
	    this.setInputsInline(true);
	},
	/**
	 * Return all variables referenced by this block.
	 * 
	 * @return {!Array.<string>} List of variable names.
	 * @this Blockly.Block
	 */
	getVars : function()
	{
		return [this.getFieldValue('VAR')];
	},
	/**
	 * Notification that a variable is renaming. If the name matches one of this
	 * block's variables, rename it.
	 * 
	 * @param {string} oldName Previous name of variable.
	 * @param {string} newName Renamed variable.
	 * @this Blockly.Block
	 */
	renameVar : function(oldName, newName)
	{
		if(Blockly.Names.equals(oldName, this.getFieldValue('VAR'))) {
			this.setFieldValue(newName, 'VAR');
		}
	}
};

Blockly.Blocks['thymio_compare'] = {
	/**
	 * Block for comparison operator. This is a copy paste of blockly's native logic_compare block, except that this one accepts only Numbers
	 * 
	 * @this Blockly.Block
	 */
	init : function()
	{
		var OPERATORS = this.RTL ? [['=', 'EQ'], ['\u2260', 'NEQ'], ['>', 'LT'], ['\u2265', 'LTE'], ['<', 'GT'], ['\u2264', 'GTE']] : [['=', 'EQ'], ['\u2260', 'NEQ'], ['<', 'LT'], ['\u2264', 'LTE'], ['>', 'GT'], ['\u2265', 'GTE']];
		this.setHelpUrl(Blockly.Msg.LOGIC_COMPARE_HELPURL);
		this.setColour(Blockly.Blocks.logic.HUE);
		this.setOutput(true, 'Boolean');
		this.appendValueInput('A').setCheck('Number');
		this.appendValueInput('B').setCheck('Number').appendField(new Blockly.FieldDropdown(OPERATORS), 'OP');
		this.setInputsInline(true);
		// Assign 'this' to a variable for use in the tooltip closure below.
		var thisBlock = this;
		this.setTooltip(function()
		{
			var op = thisBlock.getFieldValue('OP');
			var TOOLTIPS = {
				'EQ' : Blockly.Msg.LOGIC_COMPARE_TOOLTIP_EQ,
				'NEQ' : Blockly.Msg.LOGIC_COMPARE_TOOLTIP_NEQ,
				'LT' : Blockly.Msg.LOGIC_COMPARE_TOOLTIP_LT,
				'LTE' : Blockly.Msg.LOGIC_COMPARE_TOOLTIP_LTE,
				'GT' : Blockly.Msg.LOGIC_COMPARE_TOOLTIP_GT,
				'GTE' : Blockly.Msg.LOGIC_COMPARE_TOOLTIP_GTE
			};
			return TOOLTIPS[op];
		});
	}
};

Blockly.Blocks['thymio_arithmetic'] = {
	/**
	 * Block for Thymio arithmetic operators.
	 * @this Blockly.Block
	 */
	init : function()
	{
		this.setColour(Blockly.Blocks.math.HUE);
		this.setHelpUrl(Blockly.Msg.THYMIO_ARITHMETIC_HELPURL);
		this.setTooltip(Blockly.Msg.THYMIO_ARITHMETIC_TOOLTIP);
		
		var operators = [];
		
		operators.push(['+', '+']);
		operators.push(['-', '-']);
		operators.push(['*', '*']);
		operators.push(['÷', '/']);
		operators.push(['mod', '%']);
		
		this.setOutput(true, 'Number');
		this.appendValueInput('A').setCheck('Number');
		this.appendValueInput('B').setCheck('Number').appendField(new Blockly.FieldDropdown(operators), 'OP');
		this.setInputsInline(true);
	}
};

Blockly.Blocks['thymio_binary'] = {
	/**
	 * Block for Thymio binary operators.
	 * @this Blockly.Block
	 */
	init : function()
	{
		this.jsonInit({
			"message0" : Blockly.Msg.THYMIO_BINARY,
			"args0" : [{
				"type" : "field_dropdown",
				"name" : "OP",
				"options" : [
					[Blockly.Msg.THYMIO_BINARY_LEFT_SHIFT, '<<'],
					[Blockly.Msg.THYMIO_BINARY_RIGHT_SHIFT, '>>'],
					[Blockly.Msg.THYMIO_BINARY_AND, '&'],
					[Blockly.Msg.THYMIO_BINARY_OR, '|'],
					[Blockly.Msg.THYMIO_BINARY_XOR, '^']]
			},{
				"type" : "input_value",
				"name" : "A",
				"check" : "Number"
			}, {
				"type" : "input_value",
				"name" : "B",
				"check": "Number"
			}]
		});
		
		this.setColour(Blockly.Blocks.math.HUE);
		this.setHelpUrl(Blockly.Msg.THYMIO_BINARY_HELPURL);
		this.setTooltip(Blockly.Msg.THYMIO_BINARY_TOOLTIP);
		this.setOutput(true, 'Number');
		this.setInputsInline(true);
	}
};

Blockly.Blocks['thymio_unary'] = {
	/**
	 * Block for Thymio unary operators.
	 * @this Blockly.Block
	 */
	init : function()
	{
		this.jsonInit({
			"message0" : Blockly.Msg.THYMIO_UNNARY,
			"args0" : [{
				"type" : "field_dropdown",
				"name" : "OP",
				"options" : [
					[Blockly.Msg.THYMIO_UNNARY_NEGATIVE, '-'],
					[Blockly.Msg.THYMIO_UNNARY_ABSOLUTE, 'abs'],
					[Blockly.Msg.THYMIO_UNNARY_BINARY_NOT, '~']]
			}, {
				"type" : "input_value",
				"name" : "VALUE",
				"check": "Number"
			}]
		});
		
		this.setColour(Blockly.Blocks.math.HUE);
		this.setHelpUrl(Blockly.Msg.THYMIO_UNARY_HELPURL);
		this.setTooltip(Blockly.Msg.THYMIO_UNARY_TOOLTIP);
		this.setOutput(true, 'Number');
	}
};

Blockly.Blocks['thymio_communication'] = {
	/**
	 * Block for controlling Thymio's communication feature
	 * @this Blockly.Block
	 */
	init : function()
	{
		this.jsonInit({
			"message0" : Blockly.Msg.THYMIO_COMMUNICATION,
			"args0" : [{
				"type" : "field_dropdown",
				"name" : "MODE",
				"options" : [
					[Blockly.Msg.THYMIO_COMMUNICATION_ENABLE, 'ENABLE'],
					[Blockly.Msg.THYMIO_COMMUNICATION_DISABLE, 'DISABLE']]
			}]
		});
		
		this.setColour(Blockly.Blocks.thymio.ACTUATORS_HUE);
		this.setHelpUrl(Blockly.Msg.THYMIO_COMMUNICATION_HELPURL);
		this.setTooltip(Blockly.Msg.THYMIO_COMMUNICATION_TOOLTIP);
		this.setPreviousStatement(true);
		this.setNextStatement(true);
	}
};
