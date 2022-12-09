cd ./scripts/
python version_compute.py
mergehex -m ../output/Basic_Bootloader_App_Signed.hex ../../Basic_Bootloader/Objects/Basic_Bootloader.hex -o ../output/App_Firm.hex