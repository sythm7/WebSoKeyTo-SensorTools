/**
 * LEAPS - Low Energy Accurate Positioning System.
 *
 * Simple user application.
 *
 * Copyright (c) 2016-2019, LEAPS. All rights reserved.
 *
 */

#include "dwm.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// Definition pour l'envoi du message vers le broker MQTT
#define DEFAULT_NB_DISTANCES_ALLOC 1
#define USR_MSG_LINE_SIZE 13

/* Thread priority */
#ifndef THREAD_APP_PRIO
#define THREAD_APP_PRIO	20
#endif /* THREAD_APP_PRIO */

/* Thread stack size */
#ifndef THREAD_APP_STACK_SIZE
#define THREAD_APP_STACK_SIZE	(3 * 1024)
#endif /* THREAD_APP_STACK_SIZE */

#define APP_ERR_CHECK(err_code)	\
do {							\
	if ((err_code) != DWM_OK)	\
		printf("err: line(%u) code(%u)", __LINE__, (err_code));\
} while (0)						\

#define MSG_INIT	\
	"\n\n"	\
	"App   :  dwm-simple\n"	\
	"Built :  " __DATE__ " " __TIME__ "\n"	\
	"\n"


// Fonction pour convertir une chaine de caracteres en liste d'unsigned int
void string_to_uint_array(const char* str, uint8_t* array, size_t length) {
    for (size_t i = 0; i < length; i++) {
        array[i] = (uint8_t) str[i];
    }
}


/**
 * Event callback
 *
 * @param[in] p_evt  Pointer to event structure
 */
void on_dwm_evt(dwm_evt_t *p_evt)
{
	int len;
	int i;
        int nb_sensors;
        int size;
        float* list_distances = NULL;
        unsigned int* list_id = NULL;

	switch (p_evt->header.id) {
            	/* New location data */
            	case DWM_EVT_LOC_READY:
                
                        // Nombre de capteurs
                        nb_sensors = 0;
                        // Taille d'allocation des deux listes, en partie basee sur le nombre de capteurs a traiter
                        size = DEFAULT_NB_DISTANCES_ALLOC;
            
                        // Liste des distances pour chaque capteur
                        list_distances = malloc(DEFAULT_NB_DISTANCES_ALLOC * sizeof(float));
                        // Liste des ID de chaque capteur
                        list_id = malloc(DEFAULT_NB_DISTANCES_ALLOC * sizeof(int));

                        if(list_distances == NULL || list_id == NULL) {
                            printf("Erreur allocation\n");
                            return;
                        }

                        // Une iteration de la boucle for correspond au traitement d'un capteur
            		for (i = 0; i < p_evt->loc.anchors.dist.cnt; ++i) {
                        
                                // Ajouter la distance pour un capteur
                                list_distances[nb_sensors] = (float)p_evt->loc.anchors.dist.dist[i] / 1000.0f;
                                // Ajouter l'ID d'un capteur
                                list_id[nb_sensors] = (unsigned int)(p_evt->loc.anchors.dist.addr[i]);

                                nb_sensors ++;
                    
                                // Reallocation
                                if(nb_sensors == size) {
                                    size += DEFAULT_NB_DISTANCES_ALLOC;
                                    float* temp_distances = realloc(list_distances, size * sizeof(float));
                                    unsigned int* temp_id = realloc(list_id, size * sizeof(int));

                                    if(temp_distances == NULL || temp_id == NULL) {
                                        printf("Erreur de re-allocation\n");
                                        free(list_distances);
                                        free(list_id);
                                        if(temp_distances != NULL) free(temp_distances);
                                        if(temp_id != NULL) free(temp_id);
                                        return;
                                    }

                                    list_distances = temp_distances;
                                    list_id = temp_id; 
                                }

            		}
            		printf("\n");

                        // Le but est de maintenant concatener tous les ID et les distances dans une seule chaine de caractere qui sera envoyee au broker MQTT
                        // (avec comme format pour chaque ligne : <ID> <DISTANCE>\n)
                        char* msg_to_send = malloc(USR_MSG_LINE_SIZE * nb_sensors * sizeof(char));
                        msg_to_send[0] = '\0';
            
                        for (int i = 0; i < nb_sensors; i++) {
                            char temp_msg[USR_MSG_LINE_SIZE];
                            // Formater la ligne de message pour chaque paire ID - DISTANCE
                            snprintf(temp_msg, USR_MSG_LINE_SIZE, "%04X %06.2f\n", list_id[i] & 0xffff, list_distances[i]);
                
                            // Concatener temp_msg a msg_to_send
                            strcat(msg_to_send, temp_msg);
                        }
                        
                        // AFFICHAGE POUR DEBUGGUER
                        printf("%s", msg_to_send);

                        // Liberer la memoire allouee
                        free(list_distances);
                        free(list_id);

                        size_t length = strlen(msg_to_send); // Longueur de la chaîne
                        uint8_t* converted_msg = malloc(length * sizeof(uint8_t));
                        
                        if (converted_msg == NULL) {
                            printf("Erreur d'allocation de memoire\n");
                            free(msg_to_send);
                            return;
                        }
                        
                        // Convertir la chaine en tableau de type uint_t*
                        string_to_uint_array(msg_to_send, converted_msg, length); 

                        free(msg_to_send);

                        switch(dwm_usr_data_write(converted_msg, length * sizeof(uint8_t), true)) {
                                    case 1 :
                                        printf("dwm_usr_data_write : unknown command or broken TLV frame\n");
                                        break;
                                    case 2 :
                                        printf("dwm_usr_data_write : internal error\n");
                                        break;
                                    case 3 :
                                        printf("dwm_usr_data_write : invalid parameter\n");
                                        break;
                                    case 4 :
                                        printf("dwm_usr_data_write : busy");
                                        break;
                                    case 5 :
                                        printf("dwm_usr_data_write : operation not permitted\n");
                                        break;
                                    default :
                                        break;
                        }
                        free(converted_msg);
                        break;

                case DWM_EVT_USR_DATA_READY:
                        len = p_evt->header.len - sizeof(dwm_evt_hdr_t);
                        if (len <= 0)
                                break;

                        printf("iot received, len=%d:", len);
                        for (i = 0; i < len; ++i) {
                                printf(" %02X", p_evt->usr_data[i]);
                        }
                        break;

                case DWM_EVT_USR_DATA_SENT:
                        printf("iot sent\n");
                        break;

                case DWM_EVT_BH_INITIALIZED_CHANGED:
                        printf("uwbmac: backhaul = %d\n", p_evt->bh_initialized);
                        break;

                case DWM_EVT_UWBMAC_JOINED_CHANGED:
                        printf("uwbmac: joined = %d\n", p_evt->uwbmac_joined);
                        break;

                default:
                        break;
          }
}

/**
 * Application thread
 *
 * @param[in] data  Pointer to user data
 */
void app_thread_entry(uint32_t data)
{
	dwm_cfg_t cfg;
	uint8_t i2cbyte;
	dwm_evt_t evt;
	int rv;
	uint8_t label[DWM_LABEL_LEN_MAX];
	uint8_t label_len = DWM_LABEL_LEN_MAX;

	/* Get node configuration */
	APP_ERR_CHECK(dwm_cfg_get(&cfg));

	/* Update rate set to 1 second, stationary update rate set to 5 seconds */
	APP_ERR_CHECK(dwm_upd_rate_set(10, 10));

	/* Sensitivity for switching between stationary and normal update rate */
	APP_ERR_CHECK(dwm_stnry_cfg_set(DWM_STNRY_SENSITIVITY_NORMAL));

	/* Register event callback */
	dwm_evt_listener_register(
			DWM_EVT_LOC_READY | DWM_EVT_USR_DATA_READY |
			DWM_EVT_BH_INITIALIZED_CHANGED |
			DWM_EVT_UWBMAC_JOINED_CHANGED, NULL);

	/* Test the accelerometer */
	i2cbyte = 0x0f;
	rv = dwm_i2c_write(0x33 >> 1, &i2cbyte, 1, true);

	if (rv == DWM_OK) {
		rv = dwm_i2c_read(0x33 >> 1, &i2cbyte, 1);

		if (rv == DWM_OK) {
			printf("Accelerometer chip ID: %u\n", i2cbyte);
		} else {
			printf("i2c: read failed (%d)\n", rv);
		}
	} else {
		printf("i2c: write failed (%d)\n", rv);
	}

	rv = dwm_label_read(label, &label_len);

	if (rv == DWM_OK) {
		printf("LABEL(len=%d):", label_len);
		for (rv = 0; rv < label_len; ++rv) {
			printf(" %02x", label[rv]);
		}
		printf("\n");
	} else {
		printf("can't read label len=%d, error %d\n", label_len, rv);
	}

	while (1) {
		/* Thread loop */
		rv = dwm_evt_wait(&evt);
		if (rv != DWM_OK) {
			printf("dwm_evt_wait, error %d\n", rv);
		} else {
			on_dwm_evt(&evt);
		}
	}
}

/**
 * Application entry point. Initialize application thread.
 *
 * @warning ONLY ENABLING OF LOCATION ENGINE OR BLE AND CREATION AND STARTING OF
 * USER THREADS CAN BE DONE IN THIS FUNCTION
 */
void dwm_user_start(void)
{
	uint8_t hndl;
	int rv;

	dwm_shell_compile();
	//Disabling ble by default as softdevice prevents debugging with breakpoints (due to priority)
	dwm_ble_compile();
	dwm_le_compile();
	dwm_serial_spi_compile();
        
	/* Create thread */
	rv = dwm_thread_create(THREAD_APP_PRIO, app_thread_entry, (void*)NULL,
			"app", THREAD_APP_STACK_SIZE, &hndl);
	APP_ERR_CHECK(rv);

	/* Start the thread */
	dwm_thread_resume(hndl);
}
