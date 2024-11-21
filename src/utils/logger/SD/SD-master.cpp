#include "utils\logger\SD\SD-lib\src\SD.h"
#include <string>
#include "utils\logger\SD\SD-master.hpp"

#include <Arduino>
class _SD {
    private:
    
     
        bool success=false;
        
        

    public:
        File reporter;
        bool init(int CE_) {
            success=SD.begin(CE_);
            return success;            
        }
        bool write_file(std::string _name_file, std::string _string){

            reporter=SD.open("reporter.txt", 'w');
            reporter.write("Trying writing on "+_name_file+"...");

            if(SD.exists(_name_file)){
                
                File myfile=SD.open(_name_file, 'w');
                if(myfile.isDirectory()){reporter.write("Error. File is a dir"); return 0;}
                myfile.write(_string);
                reporter.write("done\n");
                myfile.close();
                reporter.close();

            }else{
                reporter.write("Error. File doesn't exist\n");
                reporter.close()
                return 0;
            } 
            return 1;
        } //se true file trovato e scritto, se false file non trovato
        bool read_file(std::string _name_file){
            File file=SD.open(_name_file, 'r');
            reporter=SD.open("reporter.txt", 'w');
            reporter.write("Trying to print "+file.name()+"...");
            
            if(file){
                if(file.isDirectory()){ reporter.write("Error: the file is a dir.\n"); return 0}
                else Serial.write(file.read().to_str());
            }else {reporter.write("Error opening file\n"); return 0;}
            reporter.close();
            file.close();
            return 1
        } //stampa il contenuto del file. ritorna true se tutto ok senno no
        bool remove(std::string dir_file){

            reporter=SD.open("reporter.txt", 'w');
            File myfile=SD.open(dir_file);
            reporter.write("Removing ");
            reporter.write(myfile.name());
            reporter.write("..."); 
                       
            if(myfile.exist()){
                
                if(myfile.isDirectory()) SD.rmdir(myfile.name());
                else SD.remove(myfile.name());
                reporter.write("done\n");

            }else reporter.write("Error: file not found\n");
            reporter.close();
        } //se false, la directory o il file non sono stati eliminati correttamente
        void clear_sd(){
            
            File myfile;
            myfile=SD.open("/");
            File temp;
            reporter=SD.open("reporter.txt", 'w');
            reporter.write("Clearing sd...");
            whule(myfile){
                temp=myfile.openNextFile();
                if(myfile.isDrectory()) SD.rmdir(myfile.name()); 
                else if(myfile.name()!=reporter.name()) SD.remove(myfile.name());
                myfile=temp;
            }
            myfile.close();
            temp.close();
            reporter.write("done\n");
            reporter.close();
        } //pulisce tutta l'sd

}