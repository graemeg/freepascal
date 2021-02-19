#!/bin/bash
./cldrparser de -d./data -o./data
echo
./cldrparser es -d./data -o./data
echo
./cldrparser fr_CA -d./data -o./data
echo
./cldrparser ja -d./data -o./data
echo
./cldrparser ko -d./data -o./data
echo
./cldrparser ru -d./data -o./data
echo
./cldrparser sv -d./data -o./data
echo
./cldrparser zh -d./data -o./data

read -p "Press [Enter] key to continue ..."