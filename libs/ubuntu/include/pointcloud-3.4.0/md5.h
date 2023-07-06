
#ifndef __MD5_H__
#define __MD5_H__


#include <string>
#include <fstream>
#include <stdint.h>
#include <stdio.h>
#include <iostream>

using namespace std;

/* Type define */
typedef unsigned char byte;
typedef unsigned int uint32;

//typedef __uint8_t uint8_t;
//typedef __uint16_t uint16_t;
//typedef __uint32_t uint32_t;
//typedef __uint64_t uint64_t;

using std::string;
using std::ifstream;


/* MD5 declaration. */
class MD5 {
public:
	MD5();
	MD5(const void *input, size_t length);
	MD5(const string &str);
	MD5(ifstream &in);
	void update(const void *input, size_t length);
	void update(const string &str);
	void update(ifstream &in);
	const byte* digest();
	string toString();
	void reset();
	void get_md5(uint8_t *data);
private:
	void update(const byte *input, size_t length);
	void final();
	void transform(const byte block[64]);
	void encode(const uint32 *input, byte *output, size_t length);
	void decode(const byte *input, uint32 *output, size_t length);
	string bytesToHexString(const byte *input, size_t length);

	/* class uncopyable */
	MD5(const MD5&);
	MD5& operator=(const MD5&);
private:
	uint32 _state[4];	/* state (ABCD) */
	uint32 _count[2];	/* number of bits, modulo 2^64 (low-order word first) */
	byte _buffer[64];	/* input buffer */
	byte _digest[16];	/* message digest */
	bool _finished;		/* calculate finished ? */

	static const byte PADDING[64];	/* padding for calculate */
	static const char HEX[16];
	static const size_t BUFFER_SIZE = 4096;
};


int md5test(void);
void PrintMD5(const string &str, MD5 &md5);


#endif/*MD5_H*/
