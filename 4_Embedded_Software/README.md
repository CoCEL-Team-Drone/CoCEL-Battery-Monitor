# Embedded Software
Implemented by Sanghyun Park and Eunseon Choi.

## API
### Overview
1. System initialization
2. BQ initialization
3. Ready for sensing

### Input
```
type def struct
{
    int32_t type;
    int32_t data_1;
    int32_t data_2;
    int32_t data_3;
}UartReceiveData;
```
- Example  
`printf '1 4 10 1\n' > /dev/ttyACM0`

### Output
```
typedef struct
{
	uint32_t timestamp;
	uint32_t read_out_time;
	int16_t status;
	uint16_t data_1;
	uint16_t data_2;
	uint16_t data_3;
	uint16_t data_4;
	uint16_t data_5;
	uint16_t data_6;
	uint16_t data_7;
	int16_t data_etc_1;
	int16_t data_etc_2;
}UartTransmitData;
```
- Example  
`1000,5,0,2453,3534,4353,3452,1379,997,234`