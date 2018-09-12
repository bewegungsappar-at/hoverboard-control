#ifndef CRC_H
#define CRC_H

/* Simple public domain implementation of the standard CRC32 checksum.
 * Outputs the checksum for each file given as a command line argument.
 * Invalid file names and files that cause errors are silently skipped.
 * The program reads from stdin if it is called with no arguments. */

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>

uint32_t crc32_for_byte(uint32_t r) {
  for(int j = 0; j < 8; ++j)
    r = (r & 1? 0: (uint32_t)0xEDB88320L) ^ r >> 1;
  return r ^ (uint32_t)0xFF000000L;
}

static uint32_t crc32_table[0x100];

void crc32_populate_table() {
    for(size_t i = 0; i < 0x100; ++i)
      crc32_table[i] = crc32_for_byte(i);
}

void crc32(const void *data, size_t n_bytes, uint32_t* crc) {
  if(!*crc32_table) crc32_populate_table();
  for(size_t i = 0; i < n_bytes; ++i)
    *crc = crc32_table[(uint8_t)*crc ^ ((uint8_t*)data)[i]] ^ *crc >> 8;
}

/*

Taken from http://home.thep.lu.se/~bjorn/crc/

int main(int ac, char** av) {
  FILE *fp;
  char buf[1L << 15];
  for(int i = ac > 1; i < ac; ++i)
    if((fp = i? fopen(av[i], "rb"): stdin)) {
      uint32_t crc = 0;
      while(!feof(fp) && !ferror(fp))
        crc32(buf, fread(buf, 1, sizeof(buf), fp), &crc);
      if(!ferror(fp))
        printf("%08x%s%s\n", crc, ac > 2? "\t": "", ac > 2? av[i]: "");
      if(i)
        fclose(fp);
    }
  return 0;
}
*/

#endif