#ifndef _BTCT_H_
#define _BTCT_H_
#include <Arduino.h>

// Implementation of functions defined in BTLE standard
class BluetoothCryptoToolbox{
public:
    BluetoothCryptoToolbox();
    void printBytes(uint8_t bytes[], uint8_t length);
    void generateSubkey(uint8_t* K, uint8_t* K1, uint8_t* K2);
    void AES_CMAC ( unsigned char *key, unsigned char *input, int length,
                  unsigned char *mac );
    int f5(uint8_t DHKey[],uint8_t N_master[], uint8_t N_slave[],
            uint8_t BD_ADDR_master[], uint8_t BD_ADDR_slave[], uint8_t MacKey[], uint8_t LTK[]);
    int f6(uint8_t W[], uint8_t N1[],uint8_t N2[],uint8_t R[], uint8_t IOCap[], uint8_t A1[], uint8_t A2[], uint8_t Ex[]);
    int g2(uint8_t U[], uint8_t V[], uint8_t X[], uint8_t Y[], uint8_t out[4]);
    int ah(uint8_t k[16], uint8_t r[3], uint8_t result[3]);
    int c1(uint8_t k[16], uint8_t r[16], uint8_t pres[7], uint8_t preq[7], uint8_t iat,
			uint8_t ia[6], uint8_t rat, uint8_t ra[6], uint8_t res[16]);
    int s1(uint8_t k[16], uint8_t r1[16], uint8_t r2[16], uint8_t res[16]);
    void test();
    void testF5();
    void testF6();
    void testAh();
    void testg2();

private:
    int AES_128(uint8_t key[], uint8_t data_in[], uint8_t data_out[]);
    void leftshift_onebit(unsigned char *input,unsigned char *output);
    void xor_128(uint8_t *a, uint8_t *b, uint8_t *out);
    void padding(unsigned char *lastb, unsigned char *pad, int length);
    void reverse_bytes(uint8_t *src, uint8_t *dst, uint16_t len);
};
extern BluetoothCryptoToolbox btct;
#endif