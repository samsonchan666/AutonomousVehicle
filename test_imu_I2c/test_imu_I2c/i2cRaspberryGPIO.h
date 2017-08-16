/* 
 * File:   newfile.h
 * Author: osc
 *
 * Created on 7. august 2016, 15:44
 */

#ifndef NEWFILE_H
#define	NEWFILE_H

#ifdef	__cplusplus
extern "C"
{
#endif
using namespace std;
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <string.h>

class i2cReadWrite {
        public:
        int file_i2c;
        unsigned char buffer[60];
        int length=2;

        void setup(char * id){//"/dev/i2c-1"
          char *filename = id;
          if ((file_i2c = open(filename, O_RDWR)) < 0) {
                  printf("Failed to open the i2c bus");
                  return;
          }

          int addr = 0x5a;
          if (ioctl(file_i2c, I2C_SLAVE, addr) < 0) {
                  printf("Failed to acquire bus access and/or talk to slave.\n");
                  return;
          }
        }

        string readi2cData(){
                string res;
                if (read(file_i2c, buffer, length) != length) {
                        printf("Failed to read from the i2c bus.\n");
                }
                else {
                        printf("Data read: %s\n", buffer);
                }
                return res;
        }
        void writeBytes(){
          buffer[0] = 0x01;
          buffer[1] = 0x02;
          if (write(file_i2c, buffer, length) != length) {
                  printf("Failed to write to the i2c bus.\n");
          }
        }
};


#ifdef	__cplusplus
}
#endif

#endif	/* NEWFILE_H */

