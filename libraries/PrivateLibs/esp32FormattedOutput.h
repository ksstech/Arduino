// FormattedMessages.h - Copyright (c) Andre Maree / KSS Technologies (Pty) Ltd. 2026

#ifdef __cplusplus
extern "C" {
#endif

#define strNL "\r\n"

int sysMessage(const char * msg);
int sysMessageLn(const char * msg);

/**
 * @brief   Allocate a memory buffer of sufficient size and convert the Arduino String() into the character buffer
 * @return  pointer to the allocated buffer containing the converted String()
 * @note    MUST remember to free() the allocated memory, as pointed to by the buffer, after use
 */
char * pcStringToArray(String str) {
  int len = str.length();
  char * pcBuf = (char *) malloc(++len);
  str.toCharArray(pcBuf, len);
  return pcBuf;
}

int vMessage(const char * fmt, va_list vaList) {
  static char caBuf[256];                       // buffer to muild messages into
  vsnprintf(caBuf, sizeof(caBuf), fmt, vaList);
  return sysMessage(caBuf);
}

int fmtMessage(const char * fmt, ...) {
	va_list vaList;
	va_start(vaList, fmt);
  return vMessage(fmt, vaList);
}

int fmtMessageLn(const char * fmt, ...) {
	va_list vaList;
	va_start(vaList, fmt);
  int iRV = vMessage(fmt, vaList);
  return iRV += sysMessage(strNL);
}

/**
 * @brief
 * @return
 * @note
 */
int fmtMessageStr(const char * fmt, String str) {
  char * pcBuf = pcStringToArray(str);        // convert String to character array
  int iRV = fmtMessage(fmt, pcBuf);           // display the formatted string
  free(pcBuf);                                // free the allocated buffer
  return iRV;                                 // return the number of characters generated
}

int fmtMessage1Wire(const char * fmt, uint8_t * pU8) {
  int iRV = sysMessage(fmt);
  iRV += fmtMessage("Fam=0x%02X  ID=", pU8[0]);           // Family
  for (int i = 6; i > 0; --i) {                           // ID in reverse order of ID bytes.
    iRV += fmtMessage("%02X ", pU8[i]);                   
  }
  return iRV += fmtMessage("CRC=0x%02X", pU8[7]);   // CRC last
}

#ifdef __cplusplus
}
#endif

