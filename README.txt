BCBP (Barcode Boarding Pass) Parser 
===================================

This ROS package takes as input a string that is a decoded barcode of a flight 
ticket. It can parse the string as dictated by the BCBP protocol (IATA 
resolution 792). Individual items can be extracted and passed on as a custom 
ROS message to other nodes.

The package can also use a (mock) database to emulate information extraction 
such as the time of departure based on a ticket's flight number. To generate a 
database, run the following in your shell:

mysql -u root -p < generate_air_db.sql

