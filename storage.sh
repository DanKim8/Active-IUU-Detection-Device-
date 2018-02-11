#!/bin/bash 
while [ "true" ]
	do
	curl -X GET --header 'Accept: application/json' --header '<Authorization>' '<DatabaseURL>?last=5m' >>output.json #Store iOT database from The Things Network to a local drive 
	sleep 300 #Want to pull in every 5 minutes
done
