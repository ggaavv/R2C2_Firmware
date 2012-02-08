/*Copyright (C) 2011 by Sagar G V

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
*/

#include "delay.h"

void few_ticks()
{
	__ASM("nop");
	__ASM("nop");
	__ASM("nop");
	__ASM("nop");
	__ASM("nop");
	__ASM("nop");
	__ASM("nop");
	__ASM("nop");
	__ASM("nop");
	__ASM("nop");
	__ASM("nop");
	__ASM("nop");
	__ASM("nop");
	__ASM("nop");
	__ASM("nop");
	__ASM("nop");
	__ASM("nop");
	__ASM("nop");
	__ASM("nop");
	__ASM("nop");
	__ASM("nop");
	__ASM("nop");
	__ASM("nop");
	__ASM("nop");
	__ASM("nop");
	__ASM("nop");
	__ASM("nop");
	__ASM("nop");
	__ASM("nop");
	__ASM("nop");
	__ASM("nop");
	__ASM("nop");
	__ASM("nop");
	__ASM("nop");
	__ASM("nop");
	__ASM("nop");
	__ASM("nop");
	__ASM("nop");
	__ASM("nop");
	__ASM("nop");
	__ASM("nop");
	__ASM("nop");
	__ASM("nop");
	__ASM("nop");
	__ASM("nop");
	__ASM("nop");
	__ASM("nop");
	__ASM("nop");
	__ASM("nop");
	__ASM("nop");
	__ASM("nop");
	__ASM("nop");
	__ASM("nop");
	__ASM("nop");
	__ASM("nop");
	__ASM("nop");
	__ASM("nop");
	__ASM("nop");
	__ASM("nop");
	__ASM("nop");
	__ASM("nop");
	__ASM("nop");
	__ASM("nop");
	__ASM("nop");
	__ASM("nop");
	__ASM("nop");
	__ASM("nop");
	__ASM("nop");
	__ASM("nop");
	__ASM("nop");
	__ASM("nop");
	__ASM("nop");
	__ASM("nop");
	__ASM("nop");
	__ASM("nop");
	__ASM("nop");
	__ASM("nop");
	__ASM("nop");
	__ASM("nop");
	__ASM("nop");
	__ASM("nop");
	__ASM("nop");
	__ASM("nop");
	__ASM("nop");
	__ASM("nop");
	__ASM("nop");
	__ASM("nop");
	__ASM("nop");
	__ASM("nop");
	__ASM("nop");
	__ASM("nop");
	__ASM("nop");
	__ASM("nop");
	__ASM("nop");
}

void _delay_us(uint32_t time)
{
	uint32_t i;
	for(i=0;i<time;i++)
		few_ticks();
	
}
void _delay_ms(uint32_t time)
{
	_delay_us(1000*time);
}
void _delay_s(uint32_t time)
{
	_delay_ms(1000*time);
}

