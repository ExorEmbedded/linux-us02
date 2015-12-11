
#include <asm/setup.h>
#include <linux/init.h>
#include <linux/string.h>
#include <linux/moduleparam.h>


/* Untouched command line saved by arch-specific code. */
extern char __initdata boot_command_line[COMMAND_LINE_SIZE];


int __initdata cmdline_getoption(char* param, char* value)
{
    char* ptr;
    char* sep;

    ptr = strstr(boot_command_line, param);
    if (ptr == NULL)
        return -1;

    ptr = ptr + strlen(param)+1;
    sep = strchr(ptr, ' ');
    if (sep != NULL)
        *sep = '\0';

    strcpy(value, ptr);

    if (sep != NULL)
        *sep = ' ';

    return 0;
}

int __initdata cmdline_getint(char* name, int* value)
{
    char svalue[100];
    int result;        
    u16 tmp;

    if (cmdline_getoption(name, svalue))
        return -1;

    result = kstrtou16(svalue, 10, &tmp);
    if (result)
        return result;

    *value = tmp;

    return 0;
}
