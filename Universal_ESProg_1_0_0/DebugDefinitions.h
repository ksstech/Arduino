// Common debug macros

#define strNL "\r\n"

// Convert uSec runtime time Sec / uSec components 
#define RTsecs            ((long)(esp_timer_get_time() / 1000000))
#define RTmicros          ((long)(esp_timer_get_time() % 1000000))

// format FUNCTION, LINE, and runtime  
#define	_RL_(f)						"[%s:%d] " f, __FUNCTION__, __LINE__
#define	_RT_(f)						"%u.%03u " f, RTsecs, RTmicros
#define	_RTL_(f)					"%u.%03u [%s:%d] " f, RTsecs, RTmicros, __FUNCTION__, __LINE__

// formatted output direct to UART using ROM support  
#define RP(f,...)         esp_rom_printf(f, ##__VA_ARGS__);
#define	RPL(f, ...)				esp_rom_printf(_RL_(f), ##__VA_ARGS__)
#define	RPT(f, ...)				esp_rom_printf(_RT_(f), ##__VA_ARGS__)
#define	RPTL(f, ...)			esp_rom_printf(_RTL_(f), ##__VA_ARGS__)
