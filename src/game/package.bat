:: –onefile	        打包成一个exe文件
:: –windowed        使用窗口，无控制台
:: --hidden-import	打包指定模块，可以多次使用
:: -y               如果dist文件夹内已经存在生成文件，则不询问用户，直接覆盖
pyinstaller --onefile --windowed --hidden-import=pygame manual_control.py
