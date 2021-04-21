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
#ifndef MySensors_NodeManager_h
#define MySensors_NodeManager_h

// include Arduino header file
#include <Arduino.h>

// include NodeManager's constants
#include "NodeManager/Constants.h"

/***********************************
Include required third-party libraries
*/

// include UDP library
#ifdef MY_USE_UDP
#include <WiFiUdp.h>
#endif

// include ESP8266 library
#ifdef CHIP_ESP8266
#include <ESP8266WiFi.h>
#endif

// include MySensors library
#include <MySensors.h>

// include additional libraries based on the requested configuration
#if NODEMANAGER_TIME == ON
#include <TimeLib.h>
#endif
#if NODEMANAGER_RTC == ON
#define NODEMANAGER_TIME ON
#include <DS3232RTC.h>
#endif
#if NODEMANAGER_CONDITIONAL_REPORT == ON
#include <float.h>
#endif
#if NODEMANAGER_SD == ON
#include <SD.h>
#endif

// disable sleep and debug when running as a serial gateway
#ifdef MY_GATEWAY_SERIAL
#define FEATURE_SLEEP OFF
#endif

// define debug output macro
#if FEATURE_DEBUG == ON
#define debug(x,...)		hwDebugPrint(x, ##__VA_ARGS__)
//#define MAX_PAYLOAD (25u)
char _convBuffer[25u*2+1];
#else
#define debug(x,...)
#endif

/***********************************
Include NodeManager core code
*/
class NodeManager;
class Sensor;

/*
* List
*/
template<typename T> class List {
public:
	typedef T* iterator;
	List() {
		_internalArray = NULL;
		_endPosition = 0;
		_allocBlocks = 0;
	}
	~List() {
		delete[] _internalArray;
		_internalArray = NULL;
		_endPosition = 0;
		_allocBlocks = 0;
	}
	void push(T item) {
		if (_endPosition == _allocBlocks) _AllocOneBlock(false);
		_internalArray[_endPosition] = item;
		++_endPosition;
	}
	void pop() {
		if (_endPosition == 0) return;
		--_endPosition;
		_DeAllocOneBlock(false);
	}
	T get(int position) {
		position = position -1;
		if (position > _endPosition) position = _endPosition;
		return _internalArray[position];
	}
	void clear() {
		T* newArray = NULL;
		if (_allocBlocks > 0) newArray = new T[_allocBlocks];
		delete[] _internalArray;
		_internalArray = newArray;
		_endPosition = 0;
	}
	inline iterator begin() { return _internalArray; }
	inline iterator end() { return _internalArray + _endPosition; }
	inline bool empty() { return (_endPosition == 0); }
	inline int size() { return _endPosition; }
	void allocateBlocks(int alloc) {
		_allocBlocks = alloc;
		T* newArray = new T[_allocBlocks];
		for (int i = 0; i < _endPosition; ++i) newArray[i] = _internalArray[i];
		delete[] _internalArray;
		_internalArray = newArray;
	}

private:
	T* _internalArray;
	int _endPosition;
	int _allocBlocks;
	void _AllocOneBlock(bool shiftItems) {
		++_allocBlocks;
		T* newArray = new T[_allocBlocks];
		for (int i = 0; i < _endPosition; ++i) newArray[shiftItems ? (i + 1) : i] = _internalArray[i];
		delete[] _internalArray;
		_internalArray = newArray;
	}
	void _DeAllocOneBlock(bool shiftItems) {
		--_allocBlocks;
		if (_allocBlocks == 0) {
			delete[] _internalArray;
			_internalArray = NULL;
			return;
		}
		T* newArray = new T[_allocBlocks];
		for (int i = 0; i < _endPosition; ++i) newArray[i] = _internalArray[shiftItems ? (i + 1) : i];
		delete[] _internalArray;
		_internalArray = newArray;
	}
};

/*
PowerManager
*/

#if FEATURE_POWER_MANAGER == ON
class PowerManager {
public:
	PowerManager(int ground_pin, int vcc_pin, int wait_time = 50);
	// to save battery the sensor can be optionally connected to two pins which will act as vcc and ground and activated on demand
	void setPowerPins(int ground_pin, int vcc_pin, int wait_time = 50);
	// if enabled the pins will be automatically powered on while awake and off during sleeping
	// turns the power pins on
	void powerOn();
	// turns the power pins on
	void powerOff();
private:
	int _vcc_pin = -1;
	int _ground_pin = -1;
	long _wait = 0;
};
#endif

/*
Timer
*/

class Timer {
public:
	Timer(NodeManager* node_manager);
	void setMode(timer_mode mode);
	timer_mode getMode();
	void setValue(int value);
	int getValue();
	// start the timer
	void start();
	// stop the timer
	void stop();
	// return true if the time is over
	bool isOver();
	// return elapsed time in seconds
	long getElapsed();
private:
	NodeManager* _node;
	timer_mode _mode = NOT_CONFIGURED;
	int _value = 0;
	bool _is_running = false;
	long _last = 0;
#if FEATURE_TIME == ON
	bool _already_reported = false;
#endif
};

/***************************************
Child: child class
*/

class Child {
public:
	Child();
	Child(Sensor* sensor, int child_id, int presentation, int type, const char* description = "");
	// set child id used to communicate with the gateway/controller
	void setChildId(int value);
	int getChildId();
	// set sensor presentation (default: S_CUSTOM)
	void setPresentation(int value);
	int getPresentation();
	// set sensor type (default: V_CUSTOM)
	void setType(int value);
	int getType();
	// set how many decimal digits to use (default: 2 for ChildFloat, 4 for ChildDouble)
	void setFloatPrecision(int value);
	// set sensor description
	void setDescription(const char* value);
	const char* getDescription();
#if FEATURE_CONDITIONAL_REPORT == ON
	// force to send an update after the configured number of minutes
	void setForceUpdateTimerValue(int value);
	// never report values below this threshold (default: FLT_MIN)
	void setMinThreshold(float value);
	// never report values above this threshold (default: FLT_MAX)
	void setMaxThreshold(float value);
	// do not report values if too close to the previous one (default: 0)
	void setValueDelta(float value);
#endif
	// send the current value to the gateway
	virtual void sendValue(bool force);
	// print the current value on a LCD display
	virtual void print(Print& device);
	// reset all the counters
	virtual void reset();
protected:
	int _samples = 0;
	Sensor* _sensor;
	int _child_id;
	int _presentation = S_CUSTOM;
	int _type = V_CUSTOM;
	int _float_precision;
	const char* _description = "";
#if FEATURE_CONDITIONAL_REPORT == ON
	Timer* _force_update_timer;
	float _min_threshold = FLT_MIN;
	float _max_threshold = FLT_MAX;
	float _value_delta = 0;
#endif
};

class ChildInt: public Child {
public:
	ChildInt(Sensor* sensor, int child_id, int presentation, int type, const char* description = "");
	void setValue(int value);
	int getValue();
	void sendValue(bool force);
	void print(Print& device);
	void reset();
private:
	int _value;
#if FEATURE_CONDITIONAL_REPORT == ON
	int _last_value = -256;
#endif
	int _total = 0;
};

class ChildFloat: public Child {
public:
	ChildFloat(Sensor* sensor, int child_id, int presentation, int type, const char* description = "");
	void setValue(float value);
	float getValue();
	void sendValue(bool force);
	void print(Print& device);
	void reset();
private:
	float _value;
#if FEATURE_CONDITIONAL_REPORT == ON
	float _last_value = -256;
#endif
	float _total = 0;
};

class ChildDouble: public Child {
public:
	ChildDouble(Sensor* sensor, int child_id, int presentation, int type, const char* description = "");
	void setValue(double value);
	double getValue();
	void sendValue(bool force);
	void print(Print& device);
	void reset();
private:
	double _value;
#if FEATURE_CONDITIONAL_REPORT == ON
	double _last_value = -256;
#endif
	double _total = 0;
};

class ChildString: public Child {
public:
	ChildString(Sensor* sensor, int child_id, int presentation, int type, const char* description = "");
	void setValue(const char* value);
	const char* getValue();
	void sendValue(bool force);
	void print(Print& device);
	void reset();
private:
	const char* _value = "";
#if FEATURE_CONDITIONAL_REPORT == ON
	const char* _last_value = "";
#endif
};

/***************************************
Sensor: generic sensor class
*/

// List class
#include "nodemanager/List.h"

// PowerManager class
#if NODEMANAGER_POWER_MANAGER == ON
#include "nodemanager/PowerManager.cpp"
#endif

// ConfigurationRequest class for OTA configuration
#if NODEMANAGER_OTA_CONFIGURATION == ON
#include "nodemanager/ConfigurationRequest.cpp"
#endif

// NodeManager class
#include "nodemanager/NodeManager.cpp"
// create the global variable nodeManager that can be called from within the sketch
extern NodeManager nodeManager;
NodeManager nodeManager;
// Sensor class
#include "nodemanager/Sensor.cpp"
// Child class
#include "nodemanager/Child.cpp"
// Timer class
#include "nodemanager/Timer.cpp"

#endif
