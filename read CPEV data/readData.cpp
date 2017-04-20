////////////////////////////////////////
// Lasted Modified by Ting-Kai Chen   //
// 2017/03/02						  //
// Note:							  //
// 	1. Seperate each lidar layer data //
////////////////////////////////////////

#include <iostream>
#include <fstream>
#include <cstring>

using namespace std;

int main(int argc, char* argv[]){
		if(argc<2){
			cout<<"usage: readData fileName"<<endl;
			return 0;
		}
		char *fileName;
		fileName=argv[1];  //File Name
		string outfile = string(argv[1])+".txt";
		// char *outfile = out;

		//sensor data
		long long int TimeStamp;
		unsigned short Version;
		char x[8*49];
		char can_rx[6];
		short str;
		bool IMU_Status;
		float IMU_Data[11];
		char SerialData[1024],SerialData2[1024];
		short SerialData_len;
		char DSRC_FLAG;
	
		FILE * f_read, * f_write;
		f_read=fopen(fileName,"rb");
		f_write=fopen(outfile.c_str(),"w");  //txt file

		fseek(f_read,0,SEEK_END);
		long size=ftell(f_read);
		fseek(f_read,0,SEEK_SET);
		long crtpos=ftell(f_read);
		
		////////////////////////////////////////////////////////////////////

		fread(&Version, 1, sizeof(unsigned short),f_read);
		fread(&x, 1, sizeof(char)*8*49, f_read);
		memset(SerialData, 0, 1024);

		////////////////////read data//////////////////////////
		int ctr = 1;
		while (crtpos!=size){

		//TimeStamp
		size_t ret = fread(&TimeStamp, 1, sizeof(long long int), f_read);
		fprintf (f_write,"%lli " ,TimeStamp);
		if(ret<=0)	return 0;

		//CAN DATA
		fread(can_rx,	1, sizeof(char)*6, f_read);
		fprintf(f_write,",%i ", static_cast<int>(can_rx[3]));
		fread(&str, 1, sizeof(short),f_read);
		float str2= float(str) / 10;
		fprintf(f_write,",%f ",str2);

		//IMU
		fread(&IMU_Status,	1, sizeof(bool),	f_read);
		if(IMU_Status)		fread(&IMU_Data,	1, sizeof(float)*11,	f_read);
		for (int i=0 ; i < 11; i++)
			fprintf (f_write,",%f ",IMU_Data[i]);
		
		//GPS
		fread(&SerialData_len,	1, sizeof(short), f_read);
		if(SerialData_len>1024)		SerialData_len = 1024;
		if(SerialData_len <=0) fprintf(f_write,"\n%s ",SerialData);
		else{
			memset(SerialData, 0, 1024);
			fread(&SerialData, 1, SerialData_len, f_read);

			/// Modify the format of GPS data ///
			string a = SerialData;
			size_t pos = a.find("\n");
			a.insert(pos, "\n");
			a.replace(a.find("\n", pos+2), 2, "\n\n");
			strcpy(SerialData, a.c_str());
			////////////

			fprintf(f_write,"\n%s ",SerialData);
		}


		//DSRC_FLAG
		fread(&DSRC_FLAG, 1, sizeof(char), f_read);

		//LIDAR
		for(char i = 0 ; i < 2 ; i++)		
		{
			for(char j = 0 ; j < 4 ; j++)	
			{
				unsigned short laserPtsCount = 0;
				fread(&laserPtsCount,	1, sizeof(laserPtsCount), f_read);
				for(int k = 0 ; k< laserPtsCount ; k++)
				{
					short ang, dist;
					fread(&ang,	1, sizeof(short),	f_read);
					fprintf(f_write,"%i, ",ang);
					fread(&dist,	1,sizeof(short),	f_read);
					fprintf(f_write,"%i, ",dist);
				}
				fprintf(f_write,"\n\n\n");
			}
		}
		/////////////////////////End///////////////////////////////

		fprintf(f_write,"\n\n");

		crtpos=ftell(f_read);
		}
		fclose(f_write);
		fclose(f_read);
		//system("pause");
		return 0;
}