/* Makefile Example 
 * - idea is: include directory
 *   that is not inside the root directory
 *   from which compilation is launched
 *   Shaun Bowman
 *   2021/06/19
 *   Purpose:
 *   gcc -E -std=c99 lcarkrpa.c > parsed.c
 *   then run parsed.c through pycparser to
 *   extract the variabel data for creation
 *   of calibration file automatically for 
 *   unit testing in CppUTest.
 *
 */

void doExample(void);
