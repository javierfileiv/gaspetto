# define rt
# mon reset
# end


# set mem inaccessible-by-default off
# target extended /dev/ttyACM0
# mon swd_scan
# attach 1
# mon connect_rst enable
# source ~/sourceCode/PyCortexMDebug/scripts/gdb.py
# svd_load ~/sourceCode/cmsis-svd/data/STMicro/STM32F411.svd
