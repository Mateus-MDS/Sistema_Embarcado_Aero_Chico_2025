/**
 * Sistema embarcado
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include "GPS_neo_6.h"

int main() {
    stdio_init_all();
    sleep_ms(2000);
    
    gps_init();
    uint32_t last_display = 0;
    
    while (true) {
        
        read_gps_data();
        
        // Verifica se GPS tem fix válido
        if (is_gps_valid()) {
            // Dados para o cartão SD
            uint32_t tempo = get_gps_time_seconds();
            double xgps = get_gps_x();
            double ygps = get_gps_y(); 
            double zgps = get_gps_z();
            double velocidade = get_gps_velocity();
            
        } else {
            printf("Aguardando GPS...\n");
        }

        //Exibição de parâmetros
        uint32_t now = to_ms_since_boot(get_absolute_time());
        if (now - last_display > 2000) {
            display_gps_data();
            last_display = now;
        }
        
        // Acima de 40 o GPS não funciona
        sleep_ms(30);
    }

}
