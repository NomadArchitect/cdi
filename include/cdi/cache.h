/*
 * Copyright (c) 2008 Antoine Kaufmann
 *
 * This program is free software. It comes without any warranty, to
 * the extent permitted by applicable law. You can redistribute it
 * and/or modify it under the terms of the Do What The Fuck You Want
 * To Public License, Version 2, as published by Sam Hocevar. See
 * http://sam.zoy.org/projects/COPYING.WTFPL for more details.
 */

#ifndef _CDI_CACHE_H_
#define _CDI_CACHE_H_

#include <stdint.h>

#include "cdi.h"

struct cdi_cache {
    /** Groesse der Blocks, die der Cache verwaltet */
    size_t      block_size;

    /* OS-Spezifische Daten folgen... */
};

struct cdi_cache_entry {
    struct cdi_cache* cache;

    /* OS-Spezifische Daten folgen... */
};

/**
 * Cache-Eintrag erstellen
 *
 * @param offset    Position auf dem Datentraeger
 * @param size      Groesse des Bereichs
 *
 * @return Handle
 */
struct cdi_cache_entry* cdi_cache_entry_new(struct cdi_cache* cache,
    uint64_t offset, size_t size);

/**
 * Cache-Eintrag freigeben
 *
 * @param entry Handle
 */
void    cdi_cache_entry_release(struct cdi_cache_entry* entry);

/**
 * Cache-Eintrag sperren
 *
 * @param entry Handle
 */
void    cdi_cache_entry_lock(struct cdi_cache_entry* entry);

/**
 * Cache-Eintrag entsperren
 *
 * @param entry Handle
 */
void    cdi_cache_entry_unlock(struct cdi_cache_entry* entry);

/**
 * Cache-Eintrag als veraendert markieren
 *
 * @param entry Handle
 */
void    cdi_cache_entry_dirty(struct cdi_cache_entry* entry);

/**
 * Einzelnen Block im Cache-Eintrag als veraendert markieren
 *
 * @param entry Handle
 * @param block Blocknummer (von 0 an)
 */
void    cdi_cache_entry_blkdirty(struct cdi_cache_entry* entry, uint64_t block);



/**
 * Pointer auf einen Block im Cache-Eintrag holen
 *
 * @param entry Handle
 * @param block Blocknummer relativ zum Eintrag (der Offset innerhalb des
 *              Eintrags wird nicht beruecksichtigt)
 *
 * @return Pointer auf den Block
 */
void*   cdi_cache_entry_block(struct cdi_cache_entry* entry, size_t block);

/**
 * Daten aus dem Cache-Eintrag lesen
 *
 * @param entry     Handle
 * @param offset    Offset relativ zum Cache-Eintrag
 * @param size      Groesse des zu lesenden Bereichs
 * @param buffer    Puffer in dem die Daten abgelegt werden sollen
 */
int     cdi_cache_entry_read(struct cdi_cache_entry* entry, size_t offset,
            size_t size, void* buffer);

/**
 * Daten in den Cache-Eintrag schreiben
 *
 * @param entry     Handle
 * @param offset    Offset relativ zum Cache-Eintrag
 * @param size      Groesse des zu schreibenden Bereichs
 * @param buffer    Puffer aus dem die Daten gelesen werden
 */
int     cdi_cache_entry_write(struct cdi_cache_entry* entry, size_t offset,
            size_t size, const void* buffer);


#endif
