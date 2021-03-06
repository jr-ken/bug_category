/*
* The MySensors Arduino library handles the wireless radio link and protocol
* between your home built sensors/actuators and HA controller of choice.
* The sensors forms a self healing radio network with optional repeaters. Each
* repeater and gateway builds a routing tables in EEPROM which keeps track of the
* network topology allowing messages to be routed to nodes.
*
* Created by Henrik Ekblad <henrik.ekblad@mysensors.org>
* Copyright (C) 2013-2017 Sensnology AB
* Full contributor list: https://github.com/mysensors/Arduino/graphs/contributors
*
* Documentation: http://www.mysensors.org
* Support Forum: http://forum.mysensors.org
*
* This program is free software; you can redistribute it and/or
* modify it under the terms of the GNU General Public License
* version 2 as published by the Free Software Foundation.
*/

/******************************************
Child: data structure for representing a Child of a Sensor
*/

#include "Child.h"

Child::Child() {
}

// constructor
Child::Child(Sensor* sensor, value_format format, int child_id, int presentation, int type, const char* description) {
	_sensor = sensor;
	_format = format;
	_child_id = child_id;
	_presentation = presentation;
	_type = type;
	_description = description;
	// set default float precision
	if (_format == FLOAT) _float_precision = 2;
	if (_format == DOUBLE) _float_precision = 4;
	// register the child with the sensor
	_sensor->registerChild(this);
#if NODEMANAGER_CONDITIONAL_REPORT == ON
	// initialize the timer for forcing updates to the gateway after a given timeframe
	_force_update_timer = new Timer();
#endif
#if NODEMANAGER_EEPROM == ON
	// define the EEPROM starting address for this child
	_eeprom_address = EEPROM_CHILD_OFFSET+_child_id*EEPROM_CHILD_SIZE;
#endif
}

// setter/getter
void Child::setChildId(int value) {
	_child_id = value;
}
int Child::getChildId() {
	return _child_id;
}
void Child::setFormat(value_format value) {
	_format = value;
	// set default float precision
	if (_format == FLOAT) _float_precision = 2;
	if (_format == DOUBLE) _float_precision = 4;
}
value_format Child::getFormat() {
	return _format;
}
void Child::setPresentation(int value) {
	_presentation = value;
}
int Child::getPresentation() {
	return _presentation;
}
void Child::setType(int value) {
	_type = value;
}
int Child::getType() {
	return _type;
}
void Child::setFloatPrecision(int value) {
	_float_precision = value;
}
void Child::setDescription(const char* value) {
	_description = value;
}
const char* Child::getDescription() {
	return _description;
}
void Child::setValueProcessing(child_processing value) {
	_value_processing = value;
}

// set the value to the child
void Child::setValue(int value) {
	_setValueNumber(value);
}
void Child::setValue(float value) {
	_setValueNumber(value);
}
void Child::setValue(double value) {
	_setValueNumber(value);
}
void Child::setValue(const char* value) {
	_value_string = value;
}

// store a new value and update the total
void Child::_setValueNumber(double value) {
	if (isnan(value)) return;
	if (_value_processing != NONE) {
		// keep track of the # of samples and total
		_total = _total + value;
		_samples++;
	}
	// process the value
	if (_value_processing == AVG) _value = _total / _samples;
	if (_value_processing == SUM) _value = _total;
	if (_value_processing == NONE) _value = value;
	// print out a debug message
	if (_format == INT) debug(PSTR(LOG_LOOP "%s(%d):SET t=%d v=%d\n"),_description,_child_id,_type,(int)_value);
	if (_format == FLOAT) debug(PSTR(LOG_LOOP "%s(%d):SET t=%d v=%d.%02d\n"),_description,_child_id,_type,(int)_value, (int)(_value*100)%100);
	if (_format == DOUBLE) debug(PSTR(LOG_LOOP "%s(%d):SET t=%d v=%d.%04d\n"),_description,_child_id,_type,(int)_value, (int)(_value*10000)%10000);
#if NODEMANAGER_EEPROM == ON
	// if the value is supposed to be persisted in EEPROM, save it
	if (_persist_value) saveValue();
#endif
}

// return the value of the child in the requested format
int Child::getValueInt() {
	return (int)_value;
}
float Child::getValueFloat() {
	return (float)_value;
}
double Child::getValueDouble(){
	return _value;
}
const char* Child::getValueString(){
	return _value_string;
}

// send the value back to the controller
void Child::sendValue(bool force) {
	if (_samples == 0) return;
#if NODEMANAGER_CONDITIONAL_REPORT == ON
	if (! force) {
		// if the value is a number
		if (_format != STRING) {
			// if below or above the thresholds, do not send the value
			if (_value < _min_threshold || _value > _max_threshold) return;
			// if the force update timer is over, send the value regardless and restart it
			if (_force_update_timer->isOver()) _force_update_timer->start();
			else {
				// if the value does not differ enough from the previous one, do not send the value
				if (_value > (_last_value - _value_delta) && _value < (_last_value + _value_delta)) {
					// keep track of the previous value
					_last_value = _value;
					return;
				}
			}
		}
		// if the value is a string
		else {
			// if a delta is configured, do not report if the string is the same as the previous one
			if (_value_delta > 0 && strcmp(_value, _last_value) == 0) {
				// keep track of the previous value
				_last_value = _value;
				return;
			}
		}
	}
	// keep track of the previous value
	if (_format != STRING) _last_value = _value;
	else _last_value_string = _value_string;
#endif
	// send the value to the gateway
	if (_format == INT) nodeManager.sendMessage(_child_id,_type,(int)_value);
	if (_format == FLOAT) nodeManager.sendMessage(_child_id,_type,(float)_value,_float_precision);
	if (_format == DOUBLE) nodeManager.sendMessage(_child_id,_type,_value,_float_precision);
	if (_format == STRING) nodeManager.sendMessage(_child_id,_type,_value_string);
	// reset the counters
	reset();
}

// print the child value to a device
void Child::print(Print& device) { 
	if (_format == INT) device.print((int)_value);
	if (_format == FLOAT) device.print((float)_value,_float_precision);
	if (_format == DOUBLE) device.print((double)_value,_float_precision);
	if (_format == STRING) device.print(_value_string);
}

// reset the counters
void Child::reset() { 
	if (_format != STRING) {
		if (_value_processing != NONE) {
			// reset the counters
			_total = 0;
			_samples = 0;
			_value = 0;
#if NODEMANAGER_EEPROM == ON
			// if the value is supposed to be persisted in EEPROM, save it
			if (_persist_value) saveValue();
#endif
		}
	} else _value_string = "";
}

#if NODEMANAGER_CONDITIONAL_REPORT == ON
// setter/getter
void Child::setForceUpdateTimerValue(int value) {
	_force_update_timer->setMode(TIME_INTERVAL);
	_force_update_timer->setValue(value*60);
}
void Child::setMinThreshold(float value) {
	_min_threshold = value;
}
void Child::setMaxThreshold(float value) {
	_max_threshold = value;
}
void Child::setValueDelta(float value) {
	_value_delta = value;
}
#endif
#if NODEMANAGER_EEPROM == ON
// setter/getter
void Child::setPersistValue(bool value) {
	_persist_value = value;
}
bool Child::getPersistValue() {
	return _persist_value;
}

// save value to EEPROM - subclass needs to implement
void Child::saveValue() {
	if (_format == STRING) return;
	// number is too large to be saved or we run out of EEPROM slots
	if (_value >= 10000 || ((_eeprom_address+EEPROM_CHILD_SIZE) > 255) ) return;
	debug(PSTR(LOG_EEPROM "%s(%d):SAVE\n"),_description,_child_id);
	// save the checksum
	nodeManager.saveToMemory(_eeprom_address+EEPROM_CHILD_CHECKSUM,0);
	// encode the sign (e.g. 0 if > 0, 1 otherwise)
	nodeManager.saveToMemory(_eeprom_address+EEPROM_CHILD_SIGN, _value >= 0 ? 0 : 1);
	// encode and save the integer value (e.g. 7240 -> int_1 = 72, int_2 = 40)
	nodeManager.saveToMemory(_eeprom_address+EEPROM_CHILD_INT_1,(int)(_value/100)%100);
	nodeManager.saveToMemory(_eeprom_address+EEPROM_CHILD_INT_2,(int)_value%100);
	// encode and save the first part of the decimal value (e.g. 7240.12 -> dec_1 = 12)
	if (_format == FLOAT || _format == DOUBLE) nodeManager.saveToMemory(_eeprom_address+EEPROM_CHILD_DEC_1,(int)(_value*100)%100);
	// encode and save the second part of the decimal value (e.g. 7240.1244 -> dec_2 = 44)
	if (_format == DOUBLE) nodeManager.saveToMemory(_eeprom_address+EEPROM_CHILD_DEC_1,(int)(_value*10000)%100);
}

// load value from EEPROM 
void Child::loadValue() { 
	if (_format == STRING) return;
	// ensure we are not going to read beyond the available EEPROM slots
	if (((_eeprom_address+EEPROM_CHILD_SIZE) > 255) ) return;
	// ensure the checksum is valid
	if (nodeManager.loadFromMemory(_eeprom_address+EEPROM_CHILD_CHECKSUM) != 0) return;
	debug(PSTR(LOG_EEPROM "%s(%d):LOAD\n"),_description,_child_id);
	// decode the integer part
	double value = 0;
	value = nodeManager.loadFromMemory(_eeprom_address+EEPROM_CHILD_INT_1)*100 + nodeManager.loadFromMemory(_eeprom_address+EEPROM_CHILD_INT_2);
	if (value == 255) return;
	// decode the sign
	if (nodeManager.loadFromMemory(_eeprom_address+EEPROM_CHILD_SIGN) == 1) value = value * -1;
	// decode the first part of the decimal value
	if (_format == FLOAT || _format == DOUBLE) value += nodeManager.loadFromMemory(_eeprom_address+EEPROM_CHILD_DEC_1)/100.0;
	// decode the second part of the decimal value
	if (_format == DOUBLE) value += nodeManager.loadFromMemory(_eeprom_address+EEPROM_CHILD_DEC_2)/10000.0;
	setValue(value);
}
#endif