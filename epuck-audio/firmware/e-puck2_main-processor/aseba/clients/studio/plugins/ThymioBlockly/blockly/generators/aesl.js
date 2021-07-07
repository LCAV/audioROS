/**
 * @fileoverview Helper functions for generating AESL for blocks.
 * @author fabian@hahn.graphics (Fabian Hahn)
 */
'use strict';

goog.provide('Blockly.AESL');

goog.require('Blockly.Generator');

/**
 * AESL code generator.
 * @type {!Blockly.Generator}
 */
Blockly.AESL = new Blockly.Generator('AESL');

Blockly.AESL.INDENT = '\t';

/**
 * List of illegal variable names.
 * This is not intended to be a security feature. Blockly is 100% client-side,
 * so bypassing this list is trivial.  This is intended to prevent users from
 * accidentally clobbering a built-in object or function.
 * @private
 */
Blockly.AESL.addReservedWords(
	'var,if,then,else,end,when,do,for,in,step,while,call,callsub,sub,onevent,emit,or,' +
	'and,not,abs,event,button,buttons,prox,motor,acc,temperature,rc5,mic,timer,math,' +
	'sound,leds,sd'
);

/**
 * Order of operation ENUMs.
 * see https://www.thymio.org/en:asebalanguage#toc8
 */
Blockly.AESL.ORDER_ATOMIC = 0;
Blockly.AESL.ORDER_INDEX = 1;
Blockly.AESL.ORDER_MINUS = 1;
Blockly.AESL.ORDER_BINARY_NOT = 2;
Blockly.AESL.ORDER_ABS = 3;
Blockly.AESL.ORDER_MULT = 4;
Blockly.AESL.ORDER_ADD = 5;
Blockly.AESL.ORDER_SHIFT = 6;
Blockly.AESL.ORDER_BINARY_AND = 7;
Blockly.AESL.ORDER_BINARY_XOR = 8;
Blockly.AESL.ORDER_BINARY_OR = 9;
Blockly.AESL.ORDER_CONDITION = 10;
Blockly.AESL.ORDER_LOGICAL_NOT = 11;
Blockly.AESL.ORDER_LOGICAL_AND = 12;
Blockly.AESL.ORDER_LOGICAL_OR = 13;
Blockly.AESL.ORDER_ASSIGNMENT = 14;
Blockly.AESL.ORDER_INCREMENT = 15;
Blockly.AESL.ORDER_NONE = 99;

/**
 * Initialise the database of variable names.
 * @param {!Blockly.Workspace} workspace Workspace to generate code from.
 */
Blockly.AESL.init = function(workspace) {
  // Create a dictionary of definitions to be printed before the code.
  Blockly.AESL.definitions_ = Object.create(null);
  // Create a dictionary mapping desired function names in definitions_
  // to actual function names (to avoid collisions with user functions).
  Blockly.AESL.functionNames_ = Object.create(null);

  if (!Blockly.AESL.variableDB_) {
    Blockly.AESL.variableDB_ =
        new Blockly.Names(Blockly.AESL.RESERVED_WORDS_);
  } else {
    Blockly.AESL.variableDB_.reset();
  }
  
  Blockly.AESL.subroutines = [];
  Blockly.AESL.arrays = [];
};

/**
 * Prepend the generated code with the variable definitions.
 * 
 * @param {string} code Generated code.
 * @return {string} Completed code.
 */
Blockly.AESL.finish = function(code)
{
	// generate global variable definitions
	if(typeof workspace !== 'undefined') {
		var variables = Blockly.Variables.allVariables(workspace); // look at workspace variables first
		for(var i = 0; i < variables.length; i++) {
			// just touch it here so variableDB will pick it up
			Blockly.AESL.variableDB_.getName(variables[i], Blockly.Variables.NAME_TYPE);
		}
	}
	
	// AESL doesn't support local variables, so what we do here is simply create new global variable
	// for each time the variable db was asked for a variable name. This is a bit of a hack since
	// it accesses the Blockly.Names internals, but as far as I can see the only way to do this
	// without changing the Blockly core.

	var defvars = [];
	var vdb = Blockly.AESL.variableDB_.dbReverse_;
	for(var property in vdb) {
	    if (Object.prototype.hasOwnProperty.call(vdb, property)) {
	    	// We abuse the variables db to generate our definitions, but subroutines use the same db
	    	// to avoid name clashes. We thus keep track of all generated subroutines and exclude them
	    	// here for variable definitions
	    	if(property in Blockly.AESL.arrays) {
	    		defvars.push('var ' + property + '[' + Blockly.AESL.arrays[property] + ']');
	    	} else if(!(property in Blockly.AESL.subroutines)) {
	    		defvars.push('var ' + property);
	    	}
	    }
	}
	
	if(defvars.length) {
		defvars.push(''); // if there is at least one variable, add a trailing newline
	}
	
	var parts = [];
	
	// variables first
	parts.push(defvars.join('\n'));
	
	// code next
	if(code.length) {
		parts.push(code);
	}
	
	// subroutines next
	for(var name in Blockly.AESL.subroutines) {
		parts.push(Blockly.AESL.subroutines[name]);
	}
	
	// all other definitions (events) last
	for(var name in Blockly.AESL.definitions_) {
		parts.push(Blockly.AESL.definitions_[name]);
	}
	
	var output = parts.join('\n\n');
	
	// replace variable hints with array access where necessary
	for(var property in vdb) {
	    if (Object.prototype.hasOwnProperty.call(vdb, property)) {
	    	var regex = new RegExp("@variable:" + property + '@', "g");
	    	
	    	if(property in Blockly.AESL.arrays) {
	    		output = output.replace(regex, property + '[0]'); // replace direct array variable access to access to first element
	    	} else if(!(property in Blockly.AESL.subroutines)) {
	    		output = output.replace(regex, property); // leave non-array access alone
	    	}
	    }
	}
	
	// Clean up temporary data.
	delete Blockly.AESL.definitions_;
	delete Blockly.AESL.functionNames_;
	Blockly.AESL.variableDB_.reset();
	Blockly.AESL.subroutines = [];
	Blockly.AESL.arrays = [];
	
	return output;
};

/**
 * Naked values are top-level blocks with outputs that aren't plugged into
 * anything.
 * 
 * @param {string}
 *            line Line of generated code.
 * @return {string} Legal line of code.
 */
Blockly.AESL.scrubNakedValue = function(line) {
  return line + '\n';
};

/**
 * Adds a handler to a Thymio event
 * @param {string} name of the event
 * @param {string} AESL code to execute for the event
 */
Blockly.AESL.addEventHandler = function(event, handler) {
	if(!Object.prototype.hasOwnProperty.call(Blockly.AESL.definitions_, 'onevent ' + event)) {
		Blockly.AESL.definitions_['onevent ' + event] = 'onevent ' + event + '\n';
	} else {
		Blockly.AESL.definitions_['onevent ' + event] += '\n';
	}
	
	Blockly.AESL.definitions_['onevent ' + event] += handler;
}

/**
 * Adds a subroutine
 * @param {string} name of the subroutine
 * @param {string} AESL code to execute for the subroutine
 */
Blockly.AESL.addSubroutine = function(name, code) {
	Blockly.AESL.subroutines[name] = 'sub ' + name + '\n' + code;
}

/**
 * Tests if a value block has a number type to be used in a condition or an assignment
 * @param {Blockly.Block} block to check
 */
Blockly.AESL.checkValueBlockType = function(block) {
	if(!block) {
		return false;
	}
	
	if(block.outputConnection.check_) {
		if(block.outputConnection.check_.indexOf('Number') < 0) {
			return false;
		}
	} else if(block.type != 'variables_get') {
		return false;
	}
	
	return true;
}

/**
 * Encode a string as a properly escaped AESL string, complete with
 * quotes.
 * @param {string} string Text to encode.
 * @return {string} AESL string.
 * @private
 */
Blockly.AESL.quote_ = function(string) {
  // TODO: This is a quick hack.  Replace with goog.string.quote
  string = string.replace(/\\/g, '\\\\')
                 .replace(/\n/g, '\\\n')
                 .replace(/'/g, '\\\'');
  return '\'' + string + '\'';
};

/**
 * Common tasks for generating AESL from blocks.
 * Handles comments for the specified block and any connected value blocks.
 * Calls any statements following this block.
 * @param {!Blockly.Block} block The current block.
 * @param {string} code The AESL code created for this block.
 * @return {string} AESL code with comments and subsequent blocks added.
 * @private
 */
Blockly.AESL.scrub_ = function(block, code) {
  var commentCode = '';
  // Only collect comments for blocks that aren't inline.
  if (!block.outputConnection || !block.outputConnection.targetConnection) {
    // Collect comment for this block.
    var comment = block.getCommentText();
    if (comment) {
      commentCode += Blockly.AESL.prefixLines(comment, '# ') + '\n';
    }
    // Collect comments for all value arguments.
    // Don't collect comments for nested statements.
    for (var x = 0; x < block.inputList.length; x++) {
      if (block.inputList[x].type == Blockly.INPUT_VALUE) {
        var childBlock = block.inputList[x].connection.targetBlock();
        if (childBlock) {
          var comment = Blockly.AESL.allNestedComments(childBlock);
          if (comment) {
            commentCode += Blockly.AESL.prefixLines(comment, '# ');
          }
        }
      }
    }
  }
  var nextBlock = block.nextConnection && block.nextConnection.targetBlock();
  var nextCode = Blockly.AESL.blockToCode(nextBlock);
  return commentCode + code + nextCode;
};
