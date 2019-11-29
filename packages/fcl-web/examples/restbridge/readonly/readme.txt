Intro
-----
This example demonstrates how to simply set up a database for export in a
read-only manner.

Execute with -h or --help to see the available command-line options.

Setup
-----

The country table (country.sql) can be used for sample data in the database
of your choice.

Set up the connection in code (method SetUpConnection), 
or in a connection.ini file with the following format:

[database]
Type=postgresql
DatabaseName=YourDatabase
Host=localhost
username=me
password=secret
port=nnnn

You can use another name as connection.ini, but then you must specify the
name with the -c or --config option.

Once started, the server will display connection info and available resources.


Things to try
-------------
The following list possible queries, using either wget or curl:
(obviously, you can do the same in a browser)

Get a list of available resources:
wget -q -O - "http://localhost:3000/metadata/"
curl -o - "http://localhost:3000/metadata/"

Same, but only the names, in compact format:
wget -q -O - "http://localhost:3000/metadata/?fl=name&fmt=csv&metadata=0"
curl -o - "http://localhost:3000/metadata/?fl=name&fmt=csv&metadata=0"


Get metadata for country table:

wget -q -O - "http://localhost:3000/metadata/country"
curl -o - "http://localhost:3000/metadata/country"

Only get fieldnames:
wget -q -O - "http://localhost:3000/metadata/country?fl=name&fmt=csv"
curl -o - "http://localhost:3000/metadata/country?fl=name&fmt=csv"

Get a list of all countries:

wget -q -O - http://localhost:3000/country
curl -o - http://localhost:3000/country

Get a list of all countries in compact format:
wget -q -O - "http://localhost:3000/country?humanreadable=0"
curl -o - "http://localhost:3000/country?humanreadable=0"

Same as previous, and skip metadata as well:
wget -O - "http://localhost:3000/country?humanreadable=0&metadata=0"
curl -o - "http://localhost:3000/country?humanreadable=0&metadata=0"

Get a list of countries in XML format:
wget -q -O - "http://localhost:3000/country?fmt=xml"
curl -o - "http://localhost:3000/country?fmt=xml"

Limit returned fields. Get a list of all ISO country codes:

wget -q -O - "http://localhost:3000/country?fl=iso"
curl -o - "http://localhost:3000/country?fl=iso";

Same, but in CSV format:
wget -q -O - "http://localhost:3000/country?fl=iso&fmt=csv"
curl -o - "http://localhost:3000/country?fl=iso&fmt=csv"

Filtering: only ISO3 codes that start with M:
wget -q -O -  "http://localhost:3000/country?iso3_gte=M&iso3_lt=N"
curl -o - "http://localhost:3000/country?iso3_gte=M&iso3_lt=N"

Paging: First page, 10 records:

wget -q -O - "http://localhost:3000/country?limit=10"
curl -o - "http://localhost:3000/country?limit=10"

Paging: Second page, 10 records:
wget -q -O - "http://localhost:3000/country?limit=10&offset=10"
curl -o - "http://localhost:3000/country?limit=10offset=10"

A full list of possibilities is available on:

https://wiki.freepascal.org/SQLDBRestBridge#Features


