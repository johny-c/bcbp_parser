# First install meta-package mysql-server
sudo apt-get install mysql-server
# This will install following packages
#mysql-client-5.5 mysql-client-core-5.5 mysql-server-5.5 mysql-server-core-5.5


# If you want to install default package from repos
#sudo apt-get install libmysqlcppconn-dev
# else do the following

# Download and extract source code for mysql connector 1.1.6
mkdir -p ~/Downloads
cd ~/Downloads
wget -o http://dev.mysql.com/get/Downloads/Connector-C++/mysql-connector-c++-1.1.6.tar.gz
tar -xvzf mysql-connector-c++-1.1.6.tar.gz

# The following is based on the INSTALL file included in the mysql-connector
# Generate Makefile
cmake .  # cmake -L to debug

# Build the libraries
make clean
make
#If all goes well, following libraries (or links) are created
#/mysql-connector-c++-1.1.6/driver/libmysqlcppconn.so
#/mysql-connector-c++-1.1.6/driver/libmysqlcppconn.so.7
#/mysql-connector-c++-1.1.6/driver/libmysqlcppconn.so.7.1.1.6
#/mysql-connector-c++-1.1.6/driver/libmysqlcppconn-static.a


# Install
#make install # This fails

# Unless you have changed the location in the configuration step, make install 
copies the header files to the directory /usr/local/include. 
The header files copied are mysql_connection.h and mysql_driver.h.

sudo mkdir -p /usr/local/include/cppconn
sudo cp -r cppconn/* /usr/local/include/cppconn/
sudo cp driver/mysql_connection.h driver/mysql_driver.h /usr/local/include/

# Again, unless you have specified otherwise, make install copies the library files 
to /usr/local/lib. The files copied are the dynamic library libmysqlcppconn.so, 
and the static library libmysqlcppconn-static.a. The extension of the dynamic 
library might be different on your system (for example, .dylib on OS X). 

sudo mkdir -p /usr/local/lib
sudo cp driver/libmysqlcppconn* /usr/local/lib/

