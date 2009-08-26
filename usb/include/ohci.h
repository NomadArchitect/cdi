/******************************************************************************
 * Copyright (c) 2009 Max Reitz                                               *
 *                                                                            *
 * Permission is hereby granted,  free of charge,  to any  person obtaining a *
 * copy of this software and associated documentation files (the "Software"), *
 * to deal in the Software without restriction,  including without limitation *
 * the rights to use,  copy, modify, merge, publish,  distribute, sublicense, *
 * and/or sell copies  of the  Software,  and to permit  persons to whom  the *
 * Software is furnished to do so, subject to the following conditions:       *
 *                                                                            *
 * The above copyright notice and this permission notice shall be included in *
 * all copies or substantial portions of the Software.                        *
 *                                                                            *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR *
 * IMPLIED, INCLUDING  BUT NOT  LIMITED TO THE WARRANTIES OF MERCHANTABILITY, *
 * FITNESS FOR A PARTICULAR  PURPOSE AND  NONINFRINGEMENT.  IN NO EVENT SHALL *
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER *
 * LIABILITY,  WHETHER IN AN ACTION OF CONTRACT,  TORT OR OTHERWISE,  ARISING *
 * FROM,  OUT OF  OR IN CONNECTION  WITH THE  SOFTWARE  OR THE  USE OR  OTHER *
 * DEALINGS IN THE SOFTWARE.                                                  *
 ******************************************************************************/

#ifndef _CDI__USB__OHCI_H
#define _CDI__USB__OHCI_H

#include <stdint.h>

#include "mempool.h"
#include "usb.h"

#define OHC_USB_RESET       0x00000000
#define OHC_USB_RESUME      0x00000040
#define OHC_USB_OPERATIONAL 0x00000080
#define OHC_USB_SUSPEND     0x000000C0

#define OHC_CTRL_CBSR  0x00000003 //Verhältnis zwischen Control und Bulk
                                  //(cbsr+1 zu 1)
#define OHC_CTRL_PLE   0x00000004 //Periodische Transfers aktivieren
#define OHC_CTRL_IE    0x00000008 //Isochronous-Transfers aktivieren
#define OHC_CTRL_CLE   0x00000010 //Control-Transfers aktivieren
#define OHC_CTRL_BLE   0x00000020 //Bulk-Transfers aktivieren
#define OHC_CTRL_HCFS  0x000000C0 //Status des HCs
#define OHC_CTRL_IR    0x00000100 //Wenn gesetzt, dann werden IRQs zum SMB
                                  //geleitet
#define OHC_CTRL_RWC   0x00000200 //Remote wakeup untersetützt
#define OHC_CTRL_RW    0x00000400 //Remote wakeup aktivieren

#define OHC_CMST_RESET 0x00000001 //Reset
#define OHC_CMST_CLF   0x00000002 //Control list filled (muss gesetzt werden,
                                  //wenn ein Eintrag zur Controlliste
                                  //hinzugefügt wird)
#define OHC_CMST_BLF   0x00000004 //Bulk list filled (muss gesetzt werden, wenn
                                  //ein Eintrag zur Bulkliste hinzugefügt wird)
#define OHC_CMST_OCR   0x00000008 //Ownership change request
#define OHC_CMST_SOC   0x00030000 //Scheduling overrun count

#define OHC_INT_SO     0x00000001 //Scheduling overrun
#define OHC_INT_WDH    0x00000002 //Write back done head
#define OHC_INT_SF     0x00000004 //Start of frame (wird bei SOF gesetzt)
#define OHC_INT_RD     0x00000008 //Resume detected (wird bei einem von einem
                                  //Gerät gesendeten Resume gesetzt)
#define OHC_INT_UE     0x00000010 //Unrecoverable error (wird bei einem HC-
                                  //Fehler gesetzt, der nichts mit USB zu tun
                                  //hat)
#define OHC_INT_FNO    0x00000020 //Frame number overflow (wird gesetzt, wenn
                                  //die Framenummer größer als 32767 und
                                  //umgebrochen wird)
#define OHC_INT_RHSC   0x00000040 //Root hub status change (wird gesetzt, wenn
                                  //sich irgendwas am Roothub geändert hat)
#define OHC_INT_OC     0x40000000 //Ownership change
#define OHC_INT_MIE    0x80000000 //(De-)Aktiviert Interrupts

#define OHC_RHA_NDP    0x000000FF //Anzahl an Rootports (1 bis 15)
#define OHC_RHA_PSM    0x00000100 //Gesetzt, wenn man die Stromzufuhr zu
                                  //einzelnen Ports unabhängig setzen kann
#define OHC_RHA_NPS    0x00000200 //Gelöscht, wenn man die Stromzufuhr zu
                                  //einzelnen Ports unabhängig setzen kann
#define OHC_RHA_DT     0x00000400 //Immer 0, zeigt so an, dass der Roothub kein
                                  //Compound device ist.
#define OHC_RHA_OCPM   0x00000800 //Gesetzt, wenn Überspannungen für jeden Port
                                  //einzeln gemeldet werden
#define OHC_RHA_NOCP   0x00001000 //Gesetzt, wenn keine Überspannungen erkannt
                                  //bzw. verhindert werden können
#define OHC_RHA_POTPGT 0xFF000000 //Power on to power good time (potpgt * 2)

#define OHC_RHS_LPS    0x00000001 //R: 0
                                  //W: Strom abdrehen
#define OHC_RHS_OCI    0x00000002 //R: Unspezifische Überspannung
#define OHC_RHS_DRWE   0x00008000 //R: Irgendwas mit Remote Wakeup Enable
                                  //W: Das aktivieren
#define OHC_RHS_LPSC   0x00010000 //R: 0
                                  //W: Strom andrehen
#define OHC_RHS_OCIC   0x00020000 //R: Bei Änderungen von OCI gesetzt
                                  //W: Feld löschen
#define OHC_RHS_CRWE   0x80000000 //W: Remote Wakeup Enable deaktivieren

#define OHC_RP_CCS     0x00000001 //R: Gerät angeschlossen
                                  //W: Deaktiviert den Port
#define OHC_RP_PES     0x00000002 //R: Port aktiviert
                                  //W: Aktiviert den Port
#define OHC_RP_PSS     0x00000004 //R: Port schläft
                                  //W: Schläfert den Port ein
#define OHC_RP_POCI    0x00000008 //R: Überspannung
                                  //W: Port aufwecken
#define OHC_RP_PRS     0x00000010 //R: Reset läuft
                                  //W: Reset treiben
#define OHC_RP_PPS     0x00000100 //R: Strom an
                                  //W: Strom andrehen
#define OHC_RP_LSDA    0x00000200 //R: Low-Speed-Gerät
                                  //W: Strom abdrehen
#define OHC_RP_CSC     0x00010000 //R: Gerät an- oder abgezogen
                                  //W: Bit löschen
#define OHC_RP_PESC    0x00020000 //R: Port (de-)aktiviert
                                  //W: Bit löschen
#define OHC_RP_PSSC    0x00040000 //R: Port wurde geweckt
                                  //W: Bit löschen
#define OHC_RP_OCIC    0x00080000 //R: Überspannungsbit verändert
                                  //W: Bit löschen
#define OHC_RP_PRSC    0x00100000 //R: Reset beendet
                                  //W: Bit löschen

struct ohci_registers {
    uint32_t hc_revision; //Revision
    volatile uint32_t hc_control;
    volatile uint32_t hc_command_status;
    volatile uint32_t hc_interrupt_status;
    volatile uint32_t hc_interrupt_enable;
    volatile uint32_t hc_interrupt_disable;
    uint32_t hc_hcca; //Adresse des HCCA
    volatile uint32_t hc_period_current_ed; //Adresse des aktuellen
                                            //Endpointdescriptors der
                                            //periodischen Transfers
    uint32_t hc_control_head_ed; //Adresse des ersten Endpointdescriptors der
                                 //Control-Transfers
    volatile uint32_t hc_control_current_ed; //Adresse des aktuellen
                                             //Endpointdescriptors der Control-
                                             //Transfers
    uint32_t hc_bulk_head_ed; //Adresse des ersten Endpointdescriptors der
                              //Bulk-Transfers
    volatile uint32_t hc_bulk_current_ed; //Adresse des aktuellen
                                          //Endpointdescriptors der Bulk-
                                          //Transfers
    volatile uint32_t hc_done_head; //Adresse des zuletzt abgearbeiteten
                                    //Transferdeskriptors, der der Done-Liste
                                    //hinzugefügt wurde
    uint32_t hc_fm_interval; //Gibt die Länge eines Frames und die maximale
                             //Paketgröße an
    volatile uint32_t hc_fm_remaining; //Verbleibende Takte des aktuellen Frames
    volatile uint32_t hc_fm_number; //Nummer des aktuellen Frames
    uint32_t hc_periodic_start; //10% von HcFmInterval, wenn HcFmInterval diesen
                                //Wert erreicht, dann haben periodische
                                //Transfers Vorrang
    uint32_t hc_ls_threshold; //Besser nicht ändern
    uint32_t hc_rh_descriptor_a;
    uint32_t hc_rh_descriptor_b;
    volatile uint32_t hc_rh_status;
    volatile uint32_t hc_rh_port_status[];
} __attribute__((packed));

struct ohci_hcca {
    uint32_t interrupt_table[32]; //Zeiger zu Interrupt-EDs
    uint16_t frame_number; //Die Nummer des aktuellen Frames
    uint16_t pad; //Wird auf 0 gesetzt, wenn frame_number neu geschrieben wird
                  //(Ausrichten an DWord-Grenzen vermutlich)
    uint32_t done_head;
    uint8_t rsvd[116];
} __attribute__((packed));

struct ohci {
    struct hci gen_hci;
    struct ohci_registers* memory;
    int root_ports;
    struct ohci_hcca* hcca;

    struct mempool* ed_pool;
    struct mempool* transfer_pool;
};

#define OHC_ED_DIR_TD  0 //Richtung im TD definiert
#define OHC_ED_DIR_OUT 1
#define OHC_ED_DIR_IN  2

struct ohci_ed {
    unsigned function : 7; //USB-Adresse des Geräts
    unsigned endpoint : 4; //Nummer des Endpoints
    unsigned direction : 2; //Richtung des Transfers
    unsigned low_speed : 1;
    unsigned skip : 1;
    unsigned format : 1; //Bei Isochronous gesetzt
    unsigned mps : 11; //Maximale Paketgröße
    unsigned user : 5; //Für uns frei verfügbar
    uint32_t td_queue_tail; //Letzter TD in der Warteschlange
    uint32_t td_queue_head; //Nächster TD in der Warteschlange
    uint32_t next_ed; //Nächster ED
} __attribute__((packed));

#define OHC_TD_DIR_SETUP 0
#define OHC_TD_DIR_OUT   1
#define OHC_TD_DIR_IN    2

struct ohci_td {
    unsigned user : 18; //Für uns frei verfügbar
    unsigned rounding : 1; //Wenn gesetzt, dann sind zu kleine Pakete nicht so
                           //schlimm
    unsigned direction : 2; //Richtung/Typ des Transfers (nur gültig, wenn das
                            //"direction"-Feld im ED 00b oder 11b ist)
    unsigned di : 3; //Gibt an, wie viele Frames der HC vor Auslösen eines IRQs
                     //warten soll
    unsigned toggle : 2; //Datatoggle (00b oder 01b - Wert muss aus dem ED
                         //ermittelt werden, 10b = DATA0, 11b = DATA1)
    unsigned error : 2; //Anzahl der aufgetretenen Fehler - bei 11b wird der
                        //Status im "condition"-Feld gespeichert
    unsigned condition : 4; //Status
    uint32_t current_buffer_pointer; //Pointer zu den Daten (bei 0 wurden alle
                                     //Daten übertragen)
    uint32_t next_td; //Nächster TD
    uint32_t buffer_end; //Letztes Datenbyte des Puffers
} __attribute__((packed));

struct cdi_driver* init_ohcd(void);
void ohci_init(struct cdi_device* cdi_hci);

#endif
