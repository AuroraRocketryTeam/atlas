

class _SD {
    private:
        
        bool success=false;
        

    public:
        File reporter;
        bool init(int CE_);
        bool write_file(std::string _name_file, std::string _string); //se true file trovato e scritto, se false file non trovato
        bool read_file(std::string _name_file); //stampa il contenuto del file. ritorna true se tutto ok senno no
        bool remove(std::string dir_file); //se false, la directory o il file non sono stati eliminati correttamente
        bool clear_sd(); //cancella tutto il contenuto dell'sd


}