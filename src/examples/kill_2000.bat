
:: https://blog.csdn.net/Da_zhenzai/article/details/132595516
for /f "tokens=5" %%a in ('netstat -ano ^| findstr :2000') do taskkill /F /PID %%a
