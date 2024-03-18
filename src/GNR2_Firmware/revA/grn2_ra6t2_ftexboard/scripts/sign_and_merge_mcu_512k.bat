cd ./scripts/
python version_compute_mcu_512k.py
mergehex -m ../output/gnr2_ra6t2_Signed_mcu_512k.hex ../../grn2_ra6t2_ftexboard_bootloader/Objects/grn2_ra6t2_bootloader_mcu_512k.hex -o ../output/App_Firm_mcu_512k.hex


