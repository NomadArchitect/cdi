/*
 * Copyright (c) 2008 The LOST Project. All rights reserved.
 *
 * This code is derived from software contributed to the LOST Project
 * by Antoine Kaufmann.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. All advertising materials mentioning features or use of this software
 *    must display the following acknowledgement:
 *     This product includes software developed by the LOST Project
 *     and its contributors.
 * 4. Neither the name of the LOST Project nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT HOLDERS OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef _EXT2_H_
#define _EXT2_H_

#include <stdint.h>
#include <stdlib.h>

struct ext2_superblock_t;

/**
 * Zentrale Struktur zur Verwaltung eines eingebundenen Dateisystemes
 */
typedef struct {
    /**
     * Funktionspointer der von Aufrufer gesetzt werden muss. Diese Funktion
     * liest Daten vom Datentraeger ein, auf dem sich das Dateisystem befindet.
     *
     * @param start Position von der an gelesen werden soll
     * @param size  Anzahl der zu lesenden Bytes
     * @param dest  Pointer auf den Puffer in dem die Daten abgelegt werden
     *              sollen
     * @param prv   Private Daten zum Zugriff auf den Datentraeger (z.B.
     *              Dateideskriptor), aus dev_private zu entnehmen
     *
     * @return 1 wenn das Lesen geklappt hat, 0 sonst
     */
    int (*dev_read)(uint64_t start, size_t size, void* dest, void* prv);

    /**
     * Funktionspointer der von Aufrufer gesetzt werden muss. Diese Funktion
     * schreibt Daten auf den Datentraeger, auf dem sich das Dateisystem
     * befindet.
     *
     * @param start Position von der an gelesen werden soll
     * @param size  Anzahl der zu lesenden Bytes
     * @param dest  Pointer auf den Puffer, aus dem die Daten gelesen werden
     *              sollen.
     * @param prv   Private Daten zum Zugriff auf den Datentraeger (z.B.
     *              Dateideskriptor), aus dev_private zu entnehmen
     *
     * @return 1 wenn das Schreiben geklappt hat, 0 sonst
     */
    int (*dev_write)(uint64_t start, size_t size, const void* source,
        void* prv);

    /// Private Daten zum Zugriff auf den Datentraeger
    void* dev_private;

    /// Superblock des Dateisystems
    struct ext2_superblock_t* sb;

    /// Blocknummer in der der erste Superblock liegt
    uint64_t sb_block;
} ext2_fs_t;

/**
 * ext2-Dateisystem einbinden. Dafuer muessen dev_read, dev_write und
 * dev_private (falls notwendig) initialisiert sein.
 *
 * @return 1 bei Erfolg, im Fehlerfall 0
 */
int ext2_fs_mount(ext2_fs_t* fs);


#include "superblock.h"
#include "blockgroup.h"
#include "inode.h"
#include "directory.h"
#include "file.h"
#include "symlink.h"

#endif
