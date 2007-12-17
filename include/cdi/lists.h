/*
 * Copyright (c) 2007 Kevin Wolf
 *
 * This program is free software. It comes without any warranty, to
 * the extent permitted by applicable law. You can redistribute it 
 * and/or modify it under the terms of the Do What The Fuck You Want 
 * To Public License, Version 2, as published by Sam Hocevar. See
 * http://sam.zoy.org/projects/COPYING.WTFPL for more details.
 */  

#ifndef _CDI_LISTS_
#define _CDI_LISTS_

#include <stdint.h>

typedef struct {
    struct list_node* anchor;
    int size;
} cdi_list_t;

/** 
 * Erzeugt eine neue Liste 
 */
cdi_list_t* cdi_list_create();

/** 
 * Gibt eine Liste frei (Werte der Listenglieder müssen bereits 
 * freigegeben sein) 
 */
void cdi_list_destroy(cdi_list_t* list);

/**
 * Fuegt ein neues Element am Anfang der Liste ein
 */
cdi_list_t* cdi_list_push(cdi_list_t* list, void* value);

/**
 * Entfernt ein Element am Anfang der Liste und gibt seinen Wert zurueck
 */
void* cdi_list_pop(cdi_list_t* list);

/**
 * Prueft, ob die Liste leer ist. Gibt 1 zurueck, wenn sie leer ist;
 * 0, wenn sie Elemente enthaelt
 */
int cdi_list_empty(cdi_list_t* list);

/**
 * Gibt ein Listenelement zurueck
 */
void* cdi_list_get(cdi_list_t* list, int index);

/**
 * Fuegt ein neues Listenelement ein
 *
 * @param index Zukuenftiger Index des neu einzufuegenden Elements
 */
cdi_list_t* cdi_list_insert(cdi_list_t* list, int index, void* value);

/**
 * Loescht ein Listenelement
 */
void* cdi_list_remove(cdi_list_t* list, int index);

/**
 * Gibt die Laenge der Liste zurueck
 */
int cdi_list_size(cdi_list_t* list);

#endif
