#pragma once

#include <tchar.h>
#include <windows.h>
#include <cstdio>

class Serial {
public:
	Serial(int port_num);
	~Serial();

	void Start(unsigned long baud_late);
	void End();
	DWORD Read(char* buffer);

private:
	HANDLE com_port_;
	int port_num_;
};