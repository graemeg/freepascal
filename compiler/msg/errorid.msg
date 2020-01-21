%%% Reordering of errorid.msg respective to errore.msg
%%% Contains all comments from errorid.msg
#   $Id: errorid.msg,v 1.124 2007/09/20 02:52:50 zaenal Exp $
#   This file is part of the Free Pascal Compiler
#   Copyright (c) 1999-2018 by the Free Pascal Development team
#
#   Indonesian Language File for Free Pascal
#   Contributed by Zaenal Mutaqin <ade999 at gmail.com>
#
#   See the file COPYING.v2, included in this distribution,
#   for details about the copyright.
#
#   This program is distributed in the hope that it will be useful,
#   but WITHOUT ANY WARRANTY; without even the implied warranty of
#   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
#

# CodePage 20127

#
# The constants are build in the following order:
# <part>_<type>_<txtidentifier>
#
# <part> is the part of the compiler the message is used
#   asmr_     assembler parsing
#   asmw_     assembler writing/binary writers
#   unit_     unit handling
#   scan_     scanner
#   parser_   parser
#   type_     type checking
#   general_  general info
#   exec_     calls to assembler, linker, binder
#
# <type> the type of the message it should normally used for
#   f_   fatal error
#   e_   error
#   w_   warning
#   n_   note
#   h_   hint
#   i_   info
#   l_   add linenumber
#   u_   used
#   t_   tried
#   c_   conditional
#   d_   debug message
#   x_   executable informations
#

#
# Umum
#
# 01023 is the last used one
#
# BeginOfTeX
% \section{Pesan kompilator umum}
% Seksi ini memberikanpesan kompilator yang tidak fatal, tapi
% menampilkan informasi berguna. Jumlah pesan itu dapat dikontrol
% dengan berbagai saklar tingkat tampilan \var{-v}.
% \begin{description}
general_t_compilername=01000_T_Kompilator: $1
% Saat saklar \var{-vt} dipakai, baris ini memberitahu anda kompilator apa
% yang dipakai.
general_d_sourceos=01001_D_OS Kompilator: $1
% Saat saklar \var{-vd} dipakai, baris ini memberitahu anda apa sistem operasi
% sumbernya.
general_i_targetos=01002_I_OS Target: $1
% Saat saklar \var{-vd} dipakai, baris ini memberitahu anda apa sistem operasi
% targetnya.
general_t_exepath=01003_T_Menggunakan path executable: $1
% Saat saklar \var{-vt} dipakai, baris ini memberitahu anda ke mana kompilator
% mencari binernya.
general_t_unitpath=01004_T_Menggunakan path unit: $1
% Saat saklar \var{-vt} dipakai, baris ini memberitahu anda ke mana kompilator
% mencari unit terkompilasinya. Anda bisa menyetel path ini dengan \var{-Fu}
general_t_includepath=01005_T_Menggunakan path include: $1
% Saat saklar \var{-vt} dipakai, baris ini memberitahu anda ke mana kompilator
% mencari file include-nya (file dipakai dalam pernyataan \var{\{\$I xxx\}}).
% Anda dapat menyetel path ini dengan opsi \var{-I}.
general_t_librarypath=01006_T_Menggunakan path librari: $1
% Saat saklar \var{-vt} dipakai, baris ini memberitahu anda ke mana kompilator
% mencari librari. Anda dapat menyetel path ini dengan opsi \var{-Fl}.
general_t_objectpath=01007_T_Menggunakan path obyek: $1
% Saat saklar \var{-vt} dipakai, baris ini memberitahu anda ke mana kompilator
% mencari file obyek yang anda link (file dipakai dalam pernyataan \var{\{\$L xxx\}}).
% Anda dapat menyetel path ini dengan opsi \var{-Fo}.
general_i_abslines_compiled=01008_I_$1 baris dikompilasi, $2 det$3
% Saat saklar \var{-vi} dipakai, kompilator melaporkan jumlah baris yang
% dikompilasi, dan waktu yang diperlukan untuk mengompilasinya (waktu nyata,
% bukan waktu program).
general_f_no_memory_left=01009_F_Memori tidak tersisa
% Kompilator kekurangan memori untuk mengkompilasi program anda. Ada beberapa
% beberapa obat untuk hal ini:
% \begin{itemize}
% \item Jika anda menggunakan opsi pembangunan kompilator, coba mengompilasi
% unit-unit berbeda secara manual.
% \item Jika anda mengompilasi program besar, pisahkan dalam units, dan 
% kompilasi ini secara terpisah.
% \item Jika kedua item sebelumnya tidak bekerja, rekompilasi kompilator dengan
% heap lebih besar (anda dapat memakai opsi \var{-Ch} untuk ini, \lihat o{Ch})
% \end{itemize}
general_i_writingresourcefile=01010_I_Menulis file Tabel String Resource: $1
% Pesan ini ditampilkan saat kompilator menuliskan file Resource String Table
% yang berisi semua string sumber untuk sebuah program.
general_e_errorwritingresourcefile=01011_E_Menulis file Tabel String Resource: $1
% Pesan ini ditampilkan saat kompilator menemukan kesalahaan ketika menulis
% file Resource String Table
general_i_fatal=01012_I_Fatal:
% Prefiks untuk Kesalahan Fatal
general_i_error=01013_I_Kesalahan:
% Prefiks untuk Kesalahan
general_i_warning=01014_I_Peringatan:
% Prefiks untuk Peringatan
general_i_note=01015_I_Catatan:
% Prefiks untuk Catatan
general_i_hint=01016_I_Petunjuk:
% Prefiks untuk Petunjuk
general_e_path_does_not_exist=01017_E_Path "$1" tidak ada
% Path yang ditetapkan tidak ada.
general_f_compilation_aborted=01018_F_Kompilasi dibatalkan
% Kompilasi sudah dibatalkan.
general_text_bytes_code=01019_byte kode
% The size of the generated executable code, in bytes.
general_text_bytes_data=01020_byte data
% The size of the generated program data, in bytes.
general_i_number_of_warnings=01021_I_$1 peringatan diterbitkan
% Jumlah peringatan yang diterbitkan selama kompilasi.
general_i_number_of_hints=01022_I_$1 petunjuk diterbitkan
% Jumlah petunjuk yang diterbitkan selama kompilasi.
general_i_number_of_notes=01023_I_$1 catatan diterbitkan
% Jumlah catatan yang diterbitkan selama kompilasi.
% \end{description}
#
# Scanner
#
# 02084 is the last used one
#
% \section{Pesan pemindai.}
% Seksi ini mendaftarkan pesan-pesan yang dikeluarkan pemindai. Pemindai 
% memelihara struktur leksikal atas file pascal, misalnya ia mencoba menemukan
% kata-kata terpakai, string, dll. Ia juga memelihara direktif dan penanganan
% kompilasi kondisional.
% \begin{description}
general_f_ioerror=01024_F_I/O error: $1
% During compilation an I/O error happened which allows no further compilation.
general_f_oserror=01025_F_Operating system error: $1
% During compilation an operating system error happened which allows no further compilation.
% \end{description}
#
# Scanner
#
# 02098 is the last used one
#
% \section{Scanner messages.}
% This section lists the messages that the scanner emits. The scanner takes
% care of the lexical structure of the pascal file, i.e. it tries to find
% reserved words, strings, etc. It also takes care of directives and
% conditional compilation handling.
% \begin{description}
scan_f_end_of_file=02000_F_Akhir file tidak diharapkan
% Ini biasanya terjadi dalam salah satu kasus berikut :
% \begin{itemize}
% \item File sumber berakhir sebelum pernyataan \var{end.} final. Ini
% terjadi umumnya saat pernyataan \var{begin} dan \var{end} tidak
% seimbang;
% \item File include berakhir ditengah-tengah sebuah pernyataan.
% \item Sebuah komentar tidak ditutup
% \end{itemize}
scan_f_string_exceeds_line=02001_F_String melebihi baris
% Kekurangan penutup ' dalam sebuah string, maka ia melibatkan
% multipel baris.
scan_f_illegal_char=02002_F_Karakter tidak benar "$1" ($2)
% Sebuah karakter yang ditemukan tidak benar dalam file input.
scan_f_syn_expected=02003_F_Sintaks salah, "$1" diharapkan tetapi "$2" ditemukan
% Ini menunjukan bahwa kompilator mengharapkan token yang berbeda daripada
% yang anda ketikan. Ini bisa terjadi hampir di manapun anda membuat sebuah
% kesalahan terhadap bahasa pascal.
scan_t_start_include_file=02004_TL_Mulai membaca file include $1
% Ketika anda menyediakan saklar \var{-vt}, kompilator memberitahu anda
% kapan ia mulai membaca sebuah file yang disertakan.
scan_w_comment_level=02005_W_Tingkat komentar $1 ditemukan
% Saat saklar \var{-vw} dipakai, maka kompilator memperingatkan anda bila
% ia menemukan komentar berulang. Komentar berulang tidak dibolehkan dalam 
% Turbo Pascal dan mungkin menjadi sumber kesalahan.
scan_n_ignored_switch=02008_N_Saklar kompilator "$1" diabaikan
% Dengan \var{-vn} hidup, kompilator memperingatkan bila ia mengabaikan saklar
scan_w_illegal_switch=02009_W_Saklar kompilator "$1" tidak benar
% Anda menyertakan saklar kompilator (misalnya \var{\{\$... \}}) yang tidak
% dikenal oleh kompilator
scan_w_switch_is_global=02010_W_Saklar global kompilator salah tempat
% Saklar kompilator salah menempatkan, anda harus ditempatkan di awal
% unit atau program.
scan_e_illegal_char_const=02011_E_Konstan char tidak benar
% Ini terjadi saat anda menetapkan karakter dengan kode ASCII-nya, seperti
% dalam \var{\#96}, tapi angka baik tidak benar ataupun di luar jangkauan.
scan_f_cannot_open_input=02012_F_Tidak bisa membuka file "$1"
% \fpc todal dapat menemukan program atau file sumber unit yang anda tetapkan
% pada baris perintah.
scan_f_cannot_open_includefile=02013_F_Tidak bisa membuka file include "$1"
% \fpc tidak bisa menemukan file sumber yang anda tetapkan dalam pernyataan
% \var{\{\$include ..\}}.
scan_e_illegal_pack_records=02015_E_Pembeda penjajaran record "$1"
% Anda menetapkan \var{\{\$PACKRECORDS n\} } atau \var{\{\$ALIGN n\} }
% dengan nilai tidak benar untuk \var{n}. Penjajaran yang benar untuk \$PACKRECORDS adalah 1, 2, 4, 8, 16, 32, C,
% NORMAL, DEFAULT, dan penjajaran yang benar untuk \$ALIGN adalah 1, 2, 4, 8, 16, 32, ON,
% OFF. Di bawah mode MacPas \$ALIGN juga mendukung MAC68K, POWER dan RESET.
scan_e_illegal_pack_enum=02016_E_Pembeda besar-minimum enum "$1" tidak benar
% Anda menetapkan \var{\{\$PACKENUM n\}} dengan nilai yang tidak benar untuk
% \var{n}. Hanya 1,2,4, NORMAL atau DEFAULT yang benar di sini.
scan_e_endif_expected=02017_E_$ENDIF diharapkan untuk $1 $2 didefinisikan dalam $3 baris $4
% Pernyataan kompilasi kondisional anda tidak seimbang.
scan_e_preproc_syntax_error=02018_E_Sintaks salah saat mengurai ekspresi kompilasi kondisional
% Ini adalah kesalahan dalam ekspresi mengikuti direktori kompilator \var{\{\$if ..\}}, 
% $ifc atau $setc.
scan_e_error_in_preproc_expr=02019_E_Mengevaluasi ekspresi kompilasi kondisional
% Ini adalah kesalahan dalam ekspresi mengikuti direktori kompilator \var{\{\$if ..\}}, 
% $ifc atau $setc.
scan_w_macro_cut_after_255_chars=02020_W_Panjang isi makro dibatasi 255 karakter
% Isi dari makro tidak bisa lebih panjang dari 255 karakter.
scan_e_endif_without_if=02021_E_ENDIF tanpa IF(N)DEF
% Pernyataan \var{\{\$IFDEF ..\}} dan {\{\$ENDIF\}} anda tidak seimbang.
scan_f_user_defined=02022_F_Didefinisikan pengguna: $1
% Yang didefinisikan pengguna terjadi kesalahan fatal. Lihat juga \progref
scan_e_user_defined=02023_E_Didefinisikan pengguna: $1
% Kesalahanyang didefinisikan pengguna terjadi. Lihat juga \progref
scan_w_user_defined=02024_W_Didefinisikan pengguna: $1
% Peringtan yang didefinisikan pengguna ditemukan. Lihat juga \progref
scan_n_user_defined=02025_N_Didefinisikan pengguna: $1
% Catatan yang didefinisikan pengguna ditemukan. Lihat juga \progref
scan_h_user_defined=02026_H_Didefinisikan pengguna: $1
% Petunjuk yang didefinisikan pengguna ditemukan. Lihat juga \progref
scan_i_user_defined=02027_I_Didefinisikan pengguna: $1
% Informasi yang didefinisikan pengguna ditemukan. Lihat juga \progref
scan_e_keyword_cant_be_a_macro=02028_E_Kata kunci didefiniskan ulang sebagai makro tidak berpengaruh
% Anda tidak dapat mendefinisikan kata kunci dengan makro.
scan_f_macro_buffer_overflow=02029_F_Bufer makro berlebihan saat membaca atau melebarkan makro
% Makro anda atau hasilnya terlalu panjang bagi kompilator.
scan_w_macro_too_deep=02030_W_Pelebaran makro melebihi kedalaman 16.
% Ketika melebarkan sebuah makro, makro diulang sampai tingkat 16.
% Kompilator tidak akan melebarkannya lagi, karena ini berupa tanda bahwa
% rekursi yang digunakan.
scan_w_wrong_styled_switch=02031_W_Saklar kompilator tidak didukung dalam gaya komentar //
% Saklar kompilator harus dalam komentar gaya pascal normal.
scan_d_handling_switch=02032_DL_Penanganan saklar "$1"
% Ketika anda menyetel info debugging pada (\var{-vd}) kompilator memberitahu
% anda saatia mengevaluasi pernyataan kompilasi kondisional.
scan_c_endif_found=02033_CL_ENDIF $1 ditemukan
% Ketika anda menghidupkan pesan kondisional (\var{-vc}), kompilator memberitahu
% anda di aman ia menemukanpernyataan kondisional.
scan_c_ifdef_found=02034_CL_IFDEF $1 ditemukan, $2
% Ketika anda menghidupkan pesan kondisioinal (\var{-vc}), kompilator memberitahu
% anda di mana ia menemukan pernyataan kondisional.
scan_c_ifopt_found=02035_CL_IFOPT $1 ditemukan, $2
% Ketika anda menghidupkan pesan kondisioinal (\var{-vc}), kompilator memberitahu
% anda di mana ia menemukan pernyataan kondisional.
scan_c_if_found=02036_CL_IF $1 ditemukan, $2
% Ketika anda menghidupkan pesan kondisioinal (\var{-vc}), kompilator memberitahu
% anda di mana ia menemukan pernyataan kondisional.
scan_c_ifndef_found=02037_CL_IFNDEF $1 ditemukan, $2
% Ketika anda menghidupkan pesan kondisioinal (\var{-vc}), kompilator memberitahu
% anda di mana ia menemukan pernyataan kondisional.
scan_c_else_found=02038_CL_ELSE $1 ditemukan, $2
% Ketika anda menghidupkan pesan kondisioinal (\var{-vc}), kompilator memberitahu
% anda di mana ia menemukan pernyataan kondisional.
scan_c_skipping_until=02039_CL_Melewati sampai...
% Ketika anda menghidupkan pesan kondisioinal (\var{-vc}), kompilator memberitahu
% anda di mana ia menemukan pernyataan kondisional, dan apakah ia melewati atau
% mengompilasi bagiannya.
scan_i_press_enter=02040_I_Tekan <return> untuk melanjutkan
% Saat saklar \var{-vi} dipakai, kompilator berhenti mengompilasi dan menunggu
% tombol \var{Enter} ditekan bila ia menemukan direktif
% \var{\{\$STOP\}}.
scan_w_unsupported_switch=02041_W_Saklar "$1" tidak didukung
% Saat peringatan dihidupkan (\var{-vw}) kompilator memperingatkan anda tentang
% saklar yang tidak didukung. Ini berarti bahwa saklar yang dipakai dalam Delphi
% atau Turbo Pascal, tetapi tidak dalam \fpc
scan_w_illegal_directive=02042_W_Direktif kompilator "$1" tidak benar
% Saat peringatan dihidupkan (\var{-vw}) kompilator memperingatkan anda tentang
% saklar yang tidak dikenal. Untuk daftar saklar yang dikenal, \progref
scan_t_back_in=02043_TL_Kembali dalam $1
% Saat anda memakai (\var{-vt}) kompilator memberitahu anda kapan ia selesai
% membaca sebuah file include.
scan_w_unsupported_app_type=02044_W_Tipe aplikasi tidak didukung: "$1"
% Anda mendapat peringatan ini, jika anda menetapkan tipe aplikasi tidak dikenal
% dengan direktif \var{\{\$APPTYPE\}}
scan_w_app_type_not_support=02045_W_APPTYPE tidak didukung oleh OS target
% Direktif \var{\{\$APPTYPE\}} hanya didukung oleh sistem operasi tertentu.
scan_w_description_not_support=02046_W_DESCRIPTION tidak didukung oleh OS target
% Direktif \var{\{\$DESCRIPTION\}} tidak didukung pada OS target ini
scan_n_version_not_support=02047_N_VERSION tidak didukung oleh OS target
% Direktif \var{\{\$VERSION\}} tidak didukung pada OS target ini
scan_n_only_exe_version=02048_N_VERSION hanya untuk EXE atau DLL
% Direktif \var{\{\$VERSION\}} hanya dipakai untuk executable atau sumber DLL.
scan_w_wrong_version_ignored=02049_W_Format salah untuk direktif VERSION "$1"
% Format direktif \var{\{\$VERSION\}} adalah versiutama.versiminor
% di mana versiutama dan versiminor adalah word.
scan_e_illegal_asmmode_specifier=02050_E_Gaya assembler yang ditetapkan "$1" tidak benar
% Ketika anda menetapkan mode assembler dengan \var{\{\$ASMMODE xxx\}}
% kompilator tidak mengenal mode yang anda tetapkan.
scan_w_no_asm_reader_switch_inside_asm=02051_W_Saklar pembaca ASM tidak mungkin di dalam pernyataan asm, "$1" hanya efektif untuk next
% Tidak mungkin untuk beralih dari satu pembaca assembler ke yang lainnya
% di dalam blok assembler. Pembaca baru hanya akan dipakai untuk pernyataan
% assembler next saja.
scan_e_wrong_switch_toggle=02052_E_Saklar toggle salah, gunakan ON/OFF atau +/-
% Anda perlu menggunakan ON atau OFF atau + atau - untuk menghidup matikan saklar
scan_e_resourcefiles_not_supported=02053_E_File resource tidak didukung untuk target ini
% Target yang anda kompilasi tidak mendukung file resource.
scan_w_include_env_not_found=02054_W_Lingkungan include "$1" tidak ditemukan dalam lingkungan
% Variabel lingkungan yang disertakan tidak ditemukan dalam lingkungan, sebaliknya
% ia akan diganti dengan string kosong.
scan_e_invalid_maxfpureg_value=02055_E_Nilai batas register FPU tidak benar
% Nilai yang benar untuk direktif ini adalah 0..8 dan NORMAL/DEFAULT
scan_w_only_one_resourcefile_supported=02056_W_Hanya satu file resource didukung untuk target ini
% Target yang anda kompilasi hanya mendukung satu file resource.
% File resource pertama yang ditemukan akan dipakai, yuang lain diabaikan.
scan_w_macro_support_turned_off=02057_W_Dukungan makro telah dimatikan
% Deklarasi makro ditemukan, tapi dukungan makro sudah dimatikan,
% maka deklarasi akan diabaikan. Untuk menghidupkan dukungan makro, kompilasi dengan
% -Sm pada baris perintah atau tambah \{\$MACRO ON\} dalam sumber file
scan_e_invalid_interface_type=02058_E_Tipe antarmuka yang ditetapkan tidak benar. Yang benar COM, CORBA atau DEFAULT.
% Tipe antarmuka yang ditetapkan tidak didukung
scan_w_appid_not_support=02059_W_APPID hanya didukung pada PalmOS
% Direktif \var{\{\$APPID\}} hanya didukung jika target PalmOS.
scan_w_appname_not_support=02060_W_APPNAME hanya didukung oleh PalmOS
% Direktif \var{\{\$APPNAME\}} hanya didukung oleh target PalmOS.
scan_e_string_exceeds_255_chars=02061_E_Konstan string tidak boleh lebih panjang dari 255 karakter
% Konstan string tunggal  dapat berisi paling banyak 255 karakter. Coba untuk
% memisahkan string dalam bagian multipel lebih kecil dan tambah dengan operator +.
scan_f_include_deep_ten=02062_F_Menyertakan file include melebihi kedalaman 16.
% Ketika menyertakan file include, file sudah diulang ke tingkat 16.
% Kompilator tidak akan melebarkannya lagi, karena ini sebagai tanda bahwa
% rekursi yang dipakai.
scan_e_too_many_push=02063_F_Terlalu banyak tingkat PUSH
% Maksimum 20 tingkat dibolehkan. Kesalahan ini hanya terjadi dalam mode MacPas.
scan_e_too_many_pop=02064_E_POP tanpa didahului PUSH
% Kesalahan ini hanya terjadi dalam mode MacPas.
scan_e_error_macro_lacks_value=02065_E_Makro atau variabel waktu kompilasi "$1" tidak memiliki nilai
% Ekspresi waktu kompilasi kondisional tidak bisa dievaluasi.
scan_e_wrong_switch_toggle_default=02066_E_Saklar toggle salah, gunakan ON/OFF/DEFAULT atau +/-/*
% Anda perlu menggunakan ON atau OFF atau DEFAULT atau + atau - atau * untuk men-toggle saklar
scan_e_mode_switch_not_allowed=02067_E_Saklar mode "$1" tidak diijinkan di sini
% Saklar mode sudah ditemukan, atau dalam hal opsi -Mmacpas,
% saklar mode terjadi setelah UNIT.
scan_e_error_macro_undefined=02068_E_Variabel waktu kompilasi atau makro "$1" tidak didefinisikan.
% Ekspresi waktu kompilasi kondisional tidak bisa dievaluasi. Hanya dalam mode MacPas.
scan_e_utf8_bigger_than_65535=02069_E_Kode UTF-8 lebih besar dari 65535 ditemukan
% \fpc menangani string utf-8 secara internal sebagai widestring misalnya kode karakter dibatasi ke 65535
scan_e_utf8_malformed=02070_E_String UTF-8 salah bentuk
% String yang diberikan bukan string UTF-8 yang benar
scan_c_switching_to_utf8=02071_C_Tanda UTF-8 ditemukan, menggunakan enkode UTF-8
% Kompilator menemukan tanda enkode UTF-8 (\$ef, \$bb, \$bf) di awal file,
% maka ia menganggapnya sebagai file UTF-8
scan_e_compile_time_typeerror=02072_E_Ekspresi waktu kompilasi: Diinginkan $1 tapi didapat $2 pada $3
% Pemeriksaan tipe pada ekspresi waktu kompilasi gagal.
scan_n_app_type_not_support=02073_N_APPTYPE tidak didukung oleh OS target
% Direktif \var{\{\$APPTYPE\}} hanya didukung oleh sistem operasi tertentu.
scan_e_illegal_optimization_specifier=02074_E_Optimasi yang ditetapkan "$1" tidak benar
% Ketika anda menetapkan optimasi dengan \var{\{\$OPTIMIZATION xxx\}}
% kompilator tidak mengenali optimasi yang anda tetapkan.
scan_w_setpeflags_not_support=02075_W_SETPEFLAGS tidak didukung oleh OS target
% Direktif \var{\{\$SETPEFLAGS\}} tidak didukung oleh OS target
scan_w_imagebase_not_support=02076_W_IMAGEBASE tidak dudkung oleh OS target
% Direktif \var{\{\$IMAGEBASE\}} tidak dudkung oleh OS target
scan_w_minstacksize_not_support=02077_W_MINSTACKSIZE tidak dudkung oleh OS target
% Direktif \var{\{\$MINSTACKSIZE\}} tidak dudkung oleh OS target
scan_w_maxstacksize_not_support=02078_W_MAXSTACKSIZE tidak dudkung oleh OS target
% Direktif \var{\{\$MAXSTACKSIZE\}} tidak dudkung oleh OS target
scanner_e_illegal_warn_state=02079_E_Kondisi direktif $WARN tidak benar
% Hanya ON dan OFF bisa dipakai sebagai kondisi dengan direktif kompilator \$warn
scan_e_only_packset=02080_E_Nilai set packing tidak benar
% Hanya 0, 1, 2, 4, 8, DEFAULT dan NORMAL diijinkan sebagai parameter packset
scan_w_pic_ignored=02081_W_Direktif PIC atau saklar diabaikan
% Beberapa target seperti windows tidak mendukung PIC maka direktif PIC dan
% saklar diabaikan.
scan_w_unsupported_switch_by_target=02082_W_Saklar "$1" tidak didukung oleh target yang dipilih saat ini
% Beberapa saklar kompilator seperti \$E tidak didukung oleh semua target.
scan_w_frameworks_darwin_only=02084_W_Opsi terkait-framework hanya didukung oleh Darwin/Mac OS X
% Frameworks bukan konsep yang dikenal, atau setidaknya didukung oleh FPC, pada sistem operasi selain Darwin/Mac OS X.
scan_e_illegal_minfpconstprec=02085_E_Konstan presisi pecahan minimal "$1" tidak benar
% Presisi minimal yang benar untuk konstan floating point adalah standar, 32 dan 64, yang berarti masing-masing presisi minimal (biasanya 32 bit), 32 bit dan 64 bit.
% \end{description}
#
# Parser
#
# 03235 is the last used one
#
% \section{Pesan pengurai}
% Seksi ini mendaftarkan semua pesan pengurai. Pengurai memelihara semantik
% atas bahasa anda, misalnya ia menentukan apakah konstruksi pascal anda
% sudah benar atau tidak.
% \begin{description}
scan_w_multiple_main_name_overrides=02086_W_Overriding name of "main" procedure multiple times, was previously set to "$1"
% The name for the main entry procedure is specified more than once. Only the last
% name will be used.
scanner_w_illegal_warn_identifier=02087_W_Illegal identifier "$1" for $WARN directive
% Identifier is not known by a \var{\{\$WARN\}} compiler directive.
scanner_e_illegal_alignment_directive=02088_E_Illegal alignment directive
% The alignment directive is not valid. Either the alignment type is not known or the alignment
% value is not a power of two.
scanner_f_illegal_utf8_bom=02089_F_It is not possible to include a file that starts with an UTF-8 BOM in a module that uses a different code page
% All source code that is part of a single compilation entity (program, library, unit) must be encoded
% in the same code page
scanner_w_directive_ignored_on_target=02090_W_Directive "$1" is ignored for the current target platform
% Some directives are ignored for certain targets, such as changing the
% packrecords and packenum settings on managed platforms.
scan_w_unavailable_system_codepage=02091_W_Current system codepage "$1" is not available for the compiler. Switching default codepage back to "$2".
% The current system codepage is not known by the compiler.
% The compiler is compiled with support for several codepages built-in.
% The codepage of the operation system is not in that list. You will need to recompile
% the compiler with support for this codepage.
scan_w_setpeoptflags_not_support=02092_W_SETPEOPTFLAGS is not supported by the target OS
% The \var{\{\$SETPEOPTFLAGS\}} directive is not supported by the target OS.
scan_e_illegal_peflag=02093_E_Illegal argument for SETPEFLAGS
% The given argument for SETPEFLAGS is neither a correct named value nor an
% ordinal value
scan_e_illegal_peoptflag=02094_E_Illegal argument for SETPEOPTFLAGS
% The given argument for SETPEOPTFLAGS is neither a correct named value nor an
% ordinal value
scan_e_unsupported_switch=02095_E_Directive $1 is not supported on this target
% Not all compiler directives are supported on all targets.
scan_w_invalid_stacksize=02096_W_The specified stack size is not within the valid range for the platform. Setting the stack size ignored.
% The valid range for the stack size is 1024 - 67107839 on 32-bit and 64-bit
% platforms and 1024 - 65520 on 16-bit platforms. Additionally, for Turbo Pascal 7
% compatibility reasons, specifying a stack size of 65521 on 16-bit platforms
% actually sets the stack size to 65520.
scan_w_heapmax_lessthan_heapmin=02097_W_The specified HeapMax value is smaller than the HeapMin value. Setting HeapMax ignored.
% The HeapMax value (if specified) must be greater than or equal to the HeapMin
% value. Otherwise, the HeapMax value is ignored.
scan_e_illegal_hugepointernormalization=02098_E_Illegal argument for HUGEPOINTERNORMALIZATION
% The only allowed values for HUGEPOINTERNORMALIZATION are BORLANDC, MICROSOFTC
% and WATCOMC.
% \end{description}
#
# Parser
#
# 03339 is the last used one
#
% \section{Parser messages}
% This section lists all parser messages. The parser takes care of the
% semantics of you language, i.e. it determines if your Pascal constructs
% are correct.
% \begin{description}
parser_e_syntax_error=03000_E_Pengurai - Sintaks Salah
% Kesalahan terhadap bahasa Turbo Pascal ditemukan. Ini terjadi biasanya
% saat karakter tidak benar ditemukan dalam file sumber.
parser_e_dont_nest_interrupt=03004_E_Prosedur INTERRUPT tidak bisa diulang
% Prosedur \var{INTERRUPT} harus global.
parser_w_proc_directive_ignored=03005_W_Tipe prosedur "$1" diabaikan
% Direktif procedure yang dietapkan diabaikan oleh program FPC.
parser_e_no_overload_for_all_procs=03006_E_Tidak semua deklarasi "$1" dideklarasikan dengan OVERLOAD
% Ketika anda ingin menggunakan direktif \var{OVERLOAD}, maka semua
% deklarasi perlu menetapkan \var{OVERLOAD}.
parser_e_export_name_double=03008_E_Duplikasi nama fungsi yang diekspor "$1"
% Nama-nama fungsi yang diekspor di dalam DLL tertentu semuanya harus berbeda
parser_e_export_ordinal_double=03009_E_Duplikasi indeks fungsi yang diekspor $1
% Indeks fungsi yang diekspor di dalam DLL tertentu semuanya harus berbeda
parser_e_export_invalid_index=03010_E_Indeks untuk fungsi yang diekspor tidak benar
% Indeks fungsi DLL harus dalam jangkauan \var{1..\$FFFF}
parser_w_parser_reloc_no_debug=03011_W_DLL yang bisa direlokasi atau info debug executable $1 tidak bekerja, dimatikan.
% It is currently not possible to include debug information in a relocatable DLL.
parser_w_parser_win32_debug_needs_WN=03012_W_Untuk membolehkan men-debug pada kode win32 anda perlu mematikan relokasi dengan opsi -WN
% Info stab salah untuk DLL atau EXE yang bisa direlokasi, gunakan -WN
% jika anda ingin men-debug win32 executable.
parser_e_constructorname_must_be_init=03013_E_Nama konstruktor harus INIT
% Anda mendeklarasikan konstruktor obyek dengan nama yang bukan \var{init}, dan
% saklar \var{-Ss} dipakai. Lihat saklar \var{-Ss} (\seeo{Ss}).
parser_e_destructorname_must_be_done=03014_E_Nama destruktor harus DONE
% Anda mendeklarasikan destruktor obyek yang bukan \var{done}, dan saklar
% \var{-Ss} dipakai. Lihat saklar \var{-Ss} (\seeo{Ss}).
parser_e_proc_inline_not_supported=03016_E_Tipe prosedur INLINE tidak didukung
% Anda mencoba untuk mengompilasi program dengan gaya inline C++, dan lupa
% menetapkan opsi \var{-Si} (\seeo{Si}). Standarnya kompilator tidak mendukung 
% gaya inline C++.
parser_w_constructor_should_be_public=03018_W_Konstruktor harus public
% Konstruktor harus dalam bagian 'public' pada sebuah deklarasi obyek (class).
parser_w_destructor_should_be_public=03019_W_Destruktor harus public
% Destruktor harus dalam bagian 'public' pada sebuah deklarasi obyek (class).
parser_n_only_one_destructor=03020_N_Class harus memiliki hanya satu destruktor
% Anda dapat mendeklarasikan hanya satu destruktor untuk sebuah kelas.
parser_e_no_local_objects=03021_E_Definisi kelas lokal tidak dibolehkan
% Kelas harus didefinisikan secara global. Tidak bisa didefinisikan di dalam
% sebuah procedure atau function
parser_f_no_anonym_objects=03022_F_Definisi kelas anonim tidak dibolehkan
% Deklarasi obyek (class) tidak benar ditemukan, misalnya obyek atau kelas
% tanpa metode yang tidak berasal dari obyek atau kelas
% Sebagai contoh:
% \begin{verbatim}
% Type o = object
%          a : longint;
%          end;
% \end{verbatim}
% akan memicu kesalahan ini.
parser_n_object_has_no_vmt=03023_N_Obyek "$1" tidak memiliki VMT
% Ini adalah sebuah catatan yang menunjukan bahwa obyek yang dideklarasikan
% tidak memiliki tabel metode virtual.
parser_e_illegal_parameter_list=03024_E_Daftar parameter tidak benar
% Anda memanggil sebuah fungsi dengan parameter yang tipenya berbeda dengan
% yang parameternya dideklarasikan pada fungsi.
parser_e_wrong_parameter_size=03026_E_Jumlah parameter salah yang ditetapkan untuk memanggil "$1"
% Ada kesalahan dalam daftar parameter pada fungsi atau prosedur,
% jumlah parameter tidak benar.
parser_e_overloaded_no_procedure=03027_E_pembeda yang di-overload "$1" bukan sebuah fungsi
% Kompilator menemukan simbol dengan nama sama seperti fungsi yang di-overload,
% tapi ia bukan sebuah fungsi yang bisa di-overload.
parser_e_overloaded_have_same_parameters=03028_E_fungsi yang di-overload memiliki daftar parameter sama
% Anda mendeklarasikan fungsi yang di-overload, tapi dengan daftar parameter yang sama.
% Fungsi yang di-overload harus memiliki setidaknya 1 parameter berbeda dalam
% deklarasinya.
parser_e_header_dont_match_forward=03029_E_Header fungsi tidak sama dengan deklarasi "$1" sebelumnya
% Anda mendeklarasikan sebuah fungsi dengan parameter yang sama tetapi
% tipe hasil atau pengubah fungsi berbeda.
parser_e_header_different_var_names=03030_E_header fungsi "$1" tidak sama yang didepan : nama var berubah $2 => $3
% Anda mendeklarasikan fungsi dalam bagian \var{interface}, atau dengan direktif
% \var{forward}, tapi mendefinisikannya dengan daftar parameter yang berbeda.
parser_n_duplicate_enum=03031_N_Nilai dalam tipe enumerasi harus membesar
% \fpc membolehkan konstruksi enumeration seperti dalam C. Contoh berikut
% mendeklarasikan dua deklarasi:
% \begin{verbatim}
% type a = (A_A,A_B,A_E:=6,A_UAS:=200);
% type a = (A_A,A_B,A_E:=6,A_UAS:=4);
% \end{verbatim}
% Deklarasi kedua akan menghasilkan kesalahan. \var{A\_UAS} perlu mempunyai 
% nilai lebih tinggi daripada \var{A\_E}, misalnya minimal 7.
parser_e_no_with_for_variable_in_other_segments=03033_E_With tidak bisa dipakai untuk variabel dalam segmen berbeda
% With menyimpan variabel secara lokal pada stack, tapi ini tidak mungkin
% jika variabel dimiliki oleh segmen lain.
parser_e_too_much_lexlevel=03034_E_pengulangan fungsi > 31
% Anda dapat mengulang definisi function hanya 31 kali.
parser_e_range_check_error=03035_E_kesalahan pemeriksaan jangkauan saat mengevaluasi konstan
% Konstan di luar dari jangkauan yang dibolehkan.
parser_w_range_check_error=03036_W_kesalahan pemeriksaan jangkauan saat mengevaluasi konstan
% Konstan di luar dari jangkauan yang dibolehkan.
parser_e_double_caselabel=03037_E_duplikasi label case
% Anda menetapkan label yang sama 2 kali dalam pernyataan \var{case}.
parser_e_case_lower_less_than_upper_bound=03038_E_Batas atas jangkauan case kurang dari batas bawah
% Batas atas dari label \var{case} kurang dari batas lebih bawah dan ini
% sia-sia
parser_e_type_const_not_possible=03039_E_Tipe konstan pada kelas atau interfaces tidak dibolehkan
% Anda tidak bisa mendeklarasikan konstan dari tipe class atau object.
parser_e_no_overloaded_procvars=03040_E_variabel fungsi pada fungsi yang di-overload tidak dibolehkan
% Anda mencoba untuk menempatkan fungsi yang di-overload ke variabel prosedural.
% Ini tidak dibolehkan
parser_e_invalid_string_size=03041_E_panjang string harus berupa nilai dari 1 sampai 255
% Panjang shortstring dalam Pascal dibatasi 255 karakter. Anda mencoba untuk
% mendeklarasikan string dengan panjang lebih rendah dari 1 atau lebih besar dari 255
parser_w_use_extended_syntax_for_objects=03042_W_gunakan sintaks diperluas NEW dan DISPOSE untuk turunan obyek
% Jika anda mempunyai pointer \var{a} ke tipe kelas, maka pernyataan
% \var{new(a)} tidak akan menginisialisasi kelas (misalnya konstruktor tidak
% dipanggil), meskipun ruang akan dialokasikan. Anda harus menerbitkan pernyataan
% \var{new(a,init)}. Ini akan mengalokasikan ruang, dan memanggil konstruktor
% obyek
parser_w_no_new_dispose_on_void_pointers=03043_W_Menggunakan NEW atau DISPOSE untuk pointer untyped sia-sia
parser_e_no_new_dispose_on_void_pointers=03044_E_Menggunakan NEW atau DISPOSE tidak mungkin untuk pointer untyped
% Anda tidak bisa menggunakan \var{new(p)} atau \var{dispose(p)} jika \var{p} adalah pointer untyped
% karena tidak ada ukuran yang terkait dengan pointer untyped.
% Diterima untuk kompatibilitas dalam mode \var{tp} dan \var{delphi}.
parser_e_class_id_expected=03045_E_pengenal kelas diharapkan
% Ini terjadi saat kompilator memindai deklarasi procedure yang berisi
% sebuah titik,
% misalnya, metode obyek atau kelas, tapi tipe di depan titik bukan
% tipe yang dikenal.
parser_e_no_type_not_allowed_here=03046_E_pengenal type tidak dibolehkan di sini
% Anda tidak bisa menggunakan sebuah type di dalam ekspresi.
parser_e_methode_id_expected=03047_E_pengenal metode diharapkan
% Pengenal ini bukan sebuah metode.
% Ini terjadi saat kompilator memindai deklarasi sebuah procedure yang berisi
% sebuah titik, misalnya nama obyek atau kelas, tapi nama prosedure bukan
% prosedur dari tipe ini.
parser_e_header_dont_match_any_member=03048_E_header fungsi tidak ada yang menyamai metode pada kelas "$1"
% Pengenal ini bukan sebuah metode.
% Ini terjadi saat kompilator memindai deklarasi prosedur yang berisi sebuah
% titik, misalnya nama obyek atau kelas, tapi nama prosedure bukan
% prosedur dari tipe ini.
parser_d_procedure_start=03049_DL_procedure/function $1
% Ketika menggunakan saklar \var{-vd}, kompilator memberitahu anda saat ia mulai
% memproses sebuah implementasi procedure atau function.
parser_e_error_in_real=03050_E_Konstan floating point tidak benar
% Kompilator mengharapkan ekspresi floating point, dan mendapatkan yang lain.
parser_e_fail_only_in_constructor=03051_E_FAIL hanya dapat dipakai dalam konstruktor
% Anda menggunakan kata kunci \var{fail} di luar metode konstruktor.
parser_e_no_paras_for_destructor=03052_E_Destruktor tidak bisa mempunyai parameter
% Anda mendeklarasikan sebuah destruktor dengan daftar parameter. Metode destruktor
% tidak bisa memiliki parameter.
parser_e_only_class_members_via_class_ref=03053_E_Only class methods, class properties and class variables can be referred with class references
% This error occurs in a situation like the following:
% \begin{verbatim}
% Type :
%    Tclass = Class of Tobject;
%
% Var C : TClass;
%
% begin
% ...
% C.free
% \end{verbatim}
% \var{Free} is not a class method and hence cannot be called with a class
% reference.
parser_e_only_class_members=03054_E_Only class methods, class properties and class variables can be accessed in class methods
% This is related to the previous error. You cannot call a method of an object
% from inside a class method. The following code would produce this error:
% \begin{verbatim}
% class procedure tobject.x;
%
% begin
%   free
% \end{verbatim}
% Because free is a normal method of a class it cannot be called from a class
% method.
parser_e_case_mismatch=03055_E_Tipe konstan dan CASE tidak sama
% Salah satu label tidak memiliki tipe yang sama seperti variabel case.
parser_e_illegal_symbol_exported=03056_E_Simbol tidak bisa diekspor dari sebuah librari
% Anda hanya bisa mengekspor prosedur dan fungsi saat anda menulis sebuah librari. Anda
% tidak bisa mengekspor variabel atau konstan.
parser_w_should_use_override=03057_W_Metode yang diturunkan tidak terlihat oleh "$1"
% Metode yang dideklarasikan \v ar{virtual} dalam kelas leluhur, harus diganti
% dalam kelas turunannya dengan direktif \var{override}. Jika anda tidak
% menetapkan direktif \var{override}, anda akan menyembunyikan metode leluhur;
% anda tidak akan menggantinya.
parser_e_nothing_to_be_overridden=03058_E_Tidak ada metode dalam kelas leluhur untuk diganti: "$1"
% Anda mencoba untuk \var{override} sebuah metode virtual pada kelas leluhur yang tidak
% ada.
parser_e_no_procedure_to_access_property=03059_E_Tidak ada anggota yang disediakan untuk mengakses properti
% Anda tidak menetapkan direktif \var{read} untuk sebuah properti.
parser_w_stored_not_implemented=03060_W_Direktif properti tersimpan belum diimplementasikan
% Direktif \var{stored} belum diimplementasikan
parser_e_ill_property_access_sym=03061_E_Simbol tidak benar untuk akses properti
% Ada kesalahan dalam direktif \var{read} atau \var{write} untuk sebuah properti
% array. Ketika anda mendeklarasikan sebuah properti array, anda hanya bisa 
% mengaksesnya dengan prosedur atau fungsi. Kode berikut akan membuat kesalahan
% \begin{verbatim}
% tmyobject = class
%   i : integer;
%   property x [i : integer]: integer read I write i;
% \end{verbatim}
%
parser_e_cant_access_protected_member=03062_E_Tidak bisa mengakses field protected pada obyek di sini
% Field yang dideklarasikan dalam seksi \var{protected} pada deklarasi obyek 
% atau kelas tidak bisa diakses dari luar modul di mana obyek itu 
% didefinisikan, atau di luar metode obyek turunannya.
parser_e_cant_access_private_member=03063_E_Tidak bisa mengakses field private pada obyek di sini
% Field yang dideklarasikan dalam seksi \var{private} pada deklarasi obyek 
% atau kelas tidak bisa diakses dari luar modul di mana obyek itu 
% didefinisikan, atau di luar metode obyek turunannya.
parser_e_overridden_methods_not_same_ret=03066_E_Metode yang diganti harus mengembalikan tipe: "$2" diganti oleh "$1" yang menghasilkan tipe lain
% Jika anda mendeklarasikan metode pengganti dalam definisi kelas, keduanya harus
% memiliki tipe hasil yang sama.
parser_e_dont_nest_export=03067_E_Fungsi yang dideklarasikan EXPORT tidak bisa diulang
% Anda tidak bisa mendeklarasikan sebuah fungsi atau prosedur di dalam function atau 
% procedure yang sudah dideklarasikan sebagai prosedur ekspor.
parser_e_methods_dont_be_export=03068_E_Method tidak bisa DIEKSPOR
% Anda tidak bisa mendeklarasikan procedure yang metode obyeknya bisa
% di-\var{export}.
parser_e_call_by_ref_without_typeconv=03069_E_Panggilan dengan var untuk arg no. $1 harus sama persis: Didapat "$2" diharapkan "$3"
% Ketika memanggil fungsi yang dideklarasikan dengan parameter \var{var}, variabel
% dalam pemanggil fungsi harus tipe yang persis sama. Tidak ada konversi tipe
% otomatis.
parser_e_no_super_class=03070_E_Kelas bukan kelas leluhur dari kelas saat ini
% Ketika memanggil metode turunan, anda mencoba untuk memanggil metode kelas
% tidak berkaitan. Anda hanya dapat memanggil metode turunan dari kelas leluhurnya.
parser_e_self_not_in_method=03071_E_SELF hanya dibolehkan dalam metode
% Anda mencoba menggunakan parameter \var{self} di luar metode obyek.
% Hanya metode yang dioper parameter \var{self}.
parser_e_generic_methods_only_in_methods=03072_E_Metode hanya bisa dalam metode lain memanggil secara langsung dengan pengenal tipe kelas
% Konstruksi seperti \var{sometype.somemethod} hanya dibolehkan dalam sebuah metode.
parser_e_illegal_colon_qualifier=03073_E_Penggunaan ':' tidak benar
% Ada menggunakan format \var{:} (titik dua) 2 kali pada ekspresi yang
% bukan ekspresi real.
parser_e_illegal_set_expr=03074_E_Kesalahan pemeriksaan jangkauan dalam set konstruktor atau duplikasi set elemen
% Deklarasi dari sebuah set berisi kesalahan. Baik salah satu elemen di luar
% jangkauan dari set type, atau dua elemen sebenarnya
% sama.
parser_e_pointer_to_class_expected=03075_E_Pointer pada obyek diharapkan
% Anda menetapkan tipe yang tidak benar dalam pernyataan \var{new}.
% Sintaks diperluas dari \var{new} memerlukan obyek sebagai parameter.
parser_e_expr_have_to_be_constructor_call=03076_E_Ekspresi harus panggilan konstruktor
% Ketika menggunakan sintaks diperluas \var{new}, anda harus menetapkan metode
% konstruktor dari obyek yang sedang anda coba buat. Prosedur yang anda tetapkan
% bukan sebuah konstruktor.
parser_e_expr_have_to_be_destructor_call=03077_E_Ekspresi harus panggilan destruktor
% Ketika menggunakan sintaks diperluas \var{dispose}, anda harus menetapkan
% metode destructor dari obyek yang sedang anda coba dispose.
% Prosedure yang anda tetapkan bukan sebuah destruktor.
parser_e_invalid_record_const=03078_E_Urutan elemen record tidak benar
% Ketika mendeklarasikan konstan record, anda menetapkan field dalam urutan
% yang salah.
parser_e_false_with_expr=03079_E_Tipe ekspresi harus tipe kelas atau record
% Pernyataan \var{with} memerlukan argumen yang tipenya adalah \var{record}
% atau \var{class}. Anda mencoba menggunakan \var{with} pada ekspresi yang 
% bukan tipe ini.
parser_e_void_function=03080_E_Prosedur tidak mengembalikan nilai
% Dalam \fpc, anda dapat menetapkan nilai balik untuk sebuah fungsi saat
% memakai pernyataan \var{exit}. Kesalahan ini terjadi ketika anda mencoba untuk
% melakukan ini dengan prosedur. Prosedure tidak bisa mengembalikan nilai.
parser_e_only_methods_allowed=03081_E_konstruktor, destruktor dan class operator harus sebuah metode
% Anda mendeklarasikan sebuah procedure sebagai destruktor, konstructor atau class operator, ketika
% prosedure bukan metode kelas.
parser_e_operator_not_overloaded=03082_E_Operator tidak di-overload
% Anda mencoba untuk menggunakan operator yang di-overload saat ia bukan tipe
% untuk di-overload.
parser_e_no_such_assignment=03083_E_Tidak mungkin untuk meng-overload penempatan tipe yang sama
% Anda tidak bisa meng-overload penempatan tipe yang dianggap oleh
% kompilator sebagai sama.
parser_e_overload_impossible=03084_E_Overload operator tidak mungkin
% Kombinasi operator, argumen dan tipe balik tidak
% kompatibel.
parser_e_no_reraise_possible=03085_E_Re-raise tidak mungkin di sini
% Anda mencoba untuk memunculkan eksepsi yang tidak diperbolehkan. Anda hanya
% dapat memunculkan eksepsi dalam sebuah blok \var{except}.
parser_e_no_new_or_dispose_for_classes=03086_E_Sintaks diperluas pada new atau dispose tidak diijinkan untuk sebuah kelas
% Anda tidak bisa membuat turunan kelas dengan sintaks yang diperluas
% \var{new}. Konstruktor harus dipakai untuk itu. Untuk alasan yang sama, anda
% tidak bisa memanggil \var{dispose} untuk dealokasi turunan kelas, destruktor
% harus dipakai untuk itu.
parser_e_procedure_overloading_is_off=03088_E_Procedure overloading dimatikan
% Ketika menggunakan saklar \var{-So}, procedure overloading dimatikan.
% Turbo Pascal tidak mendukung overload fungsi.
parser_e_overload_operator_failed=03089_E_Tidak mungkinuntuk meng-overload operator ini. Operator yang bisa di-overload terkait (bila ada) adalah: $1
% Anda mencoba untuk meng-overload sebuah operator yang tidak bisa di-overload.
% Operator berikut dapat di-overload :
% \begin{verbatim}
%    +, -, *, /, =, >, <, <=, >=, is, as, in, **, :=
% \end{verbatim}
parser_e_comparative_operator_return_boolean=03090_E_Operator komparatif harus mengembalikan nilai boolean
% Ketika meng-overload operator \var{=}, fungsi harus mengembalikan nilai
% boolean.
parser_e_only_virtual_methods_abstract=03091_E_Hanya metode virtual dapat berupa abstak
% Anda mendeklarasikan metode sebagai abstrak, ketika ia tidak dideklarasikan menjadi
% virtual.
parser_f_unsupported_feature=03092_F_Penggunaan fitur yang tidak didukung!
% Anda mencoba untuk memaksa kompilator melakukan sesuatu yang belum bisa dilakukannya.
parser_e_mix_of_classes_and_objects=03093_E_Campuran dari jenis obyek berbeda (class, object, interface, dll) tidak dibolehkan
% Anda tidak dapat menurunkan \var{objects}, \var{classes}, \var{cppclasses} dan \var{interfaces} misalnya
% sebuah kelas tidak memiliki obyek sebagai leluhurnya dan sebaliknya.
parser_w_unknown_proc_directive_ignored=03094_W_Direktif procedure tidak dikenal harus diabaikan: "$1"
% Direktif procedure yang anda tetapkan tidak dikenal.
parser_e_directive_only_one_var=03095_E_$1 can be associated with only one variable
% You cannot specify more than one variable before the \var{absolute}, \var{export}, \var{external},
% \var{weakexternal}, \var{public} and \var{cvar} directives.
% As a result, for example the following construct will provide this error:
% \begin{verbatim}
% Var Z : Longint;
%     X,Y : Longint absolute Z;
% \end{verbatim}
parser_e_absolute_only_to_var_or_const=03096_E_absolute hanya bisa dikaitkan dengan var atau const
% Alamat direktif \var{absolute} hanya bisa mengarah ke variabel atau konstan.
% Oleh karena itu, kode berikut akan menghasilkan kesalahan ini:
% \begin{verbatim}
%   Procedure X;
%
%  var p : longint absolute x;
% \end{verbatim}
%
parser_e_initialized_only_one_var=03097_E_Hanya satu variabel dapat diinisialisasi
% Anda tidak dapat menetapkan lebih dari satu variabel dengan nilai awal
% dalam mode Delphi.
parser_e_abstract_no_definition=03098_E_Metode abstract tidak boleh memiliki definisi apapun (dengan badan fungsi)
% Metode abstract hanya bisa dideklarasikan, anda tidak bisa mengimplementasikannya. Ia
% harus diganti dengan kelas turunannya.
parser_e_overloaded_must_be_all_global=03099_E_Fungsi yang di-overload ini tidak boleh lokal (harus diekspor)
% Anda mendefinisikan fungsi yang di-overload dalam bagian implementation pada sebuah
% unit, tapi tidak ada deklarasi terkait dalam bagian interface dari unit itu.
parser_w_virtual_without_constructor=03100_W_Metode virtual dipakai tanpa sebuah constructor dalam "$1"
% Jika anda mendeklarasikan obyek atau kelas yang berisi metode virtual, anda perlu
% mempunyai constructor dan destructor untuk menginisialisasinya. Kompilator
% menemukan obyek atau kelas dengan metode virtual yang tidak memiliki pasangan
% constructor/destructor.
parser_c_macro_defined=03101_CL_Makro didefinisikan: $1
% Ketika \var{-vc} dipakai, kompilator memberitahu anda saat ia mendefinisikan makro.
parser_c_macro_undefined=03102_CL_Makro tidak didefinisikan: $1
% Ketika \var{-vc} dipakai, kompilator memberitahu anda saat ia tidak mendefinisikan makro.
parser_c_macro_set_to=03103_CL_Makro $1 disetel ke $2
% Ketika \var{-vc} dipakai, kompilator memberitahu anda nilai apa yang didapat makro.
parser_i_compiling=03104_I_Mengompilasi $1
% Ketika anda menghidupkan pesan informasi (\var{-vi}), kompilator memebritahu
% anda unit apa yang sedang dikompilasi.
parser_u_parsing_interface=03105_UL_Menguraikan interface dari unit $1
% Ini memberitahu anda bahwa pembacaan interface dari unit saat ini
% dimulai
parser_u_parsing_implementation=03106_UL_Menguraikan implementation dari $1
% Ini memberitahu anda bahwa pembacaan implementation dari unit saat ini,
% librari atau program dimulai
parser_d_compiling_second_time=03107_DL_Mengompilasi $1 untuk kedua kalinya
% Ketika anda meminta pesan debug (\var{-vd}) kompilator memberitahu anda unit
% apa yang dikompilasi untuk kedua kalinya.
parser_e_no_property_found_to_override=03109_E_Tidak ada properti yang ditemukan untuk diganti
% Anda ingin mengganti sebuah properti dari kelas leluhur, saat ia ada, 
% kenyataannya, tidak ada properti seperti itu dalam kelas leluhur.
parser_e_only_one_default_property=03110_E_Hanya sati properti default yang dibolehkan
% Anda menetapkan properti sebagai \var{Default}, tapi kelas sudah mempunyai 
% properti default, dan kelas hanya dapat memiliki satu properti default.
parser_e_property_need_paras=03111_E_Properti default harus berupa properti array
% Hanya properti array dari kelas dapat dibuat menjadi properti \var{default}.
parser_e_constructor_cannot_be_not_virtual=03112_E_Konstruktor virtual hanya didukung dalam model obyek kelas
% Anda tidak bisa memiliki konstruktor virtual dalam obyek. Anda hanya memilikinya
% dalam kelas.
parser_e_no_default_property_available=03113_E_Tidak ada properti default yang tersedia
% Anda mencoba mengakses properti default pada sebuah kelas, tapi kelas ini 
% atau salah satu leluhurnya) tidak mempunyai properti default.
parser_e_cant_have_published=03114_E_Kelas tidak mempunyai seksi published, pakai saklar {$M+}
% Jika anda ingin seksi \var{published} dalam sebuah definisi class, anda harus
% menggunakan saklar \var{\{\$M+\}}, yang menghidupkan pembuatan informasi
% type.
parser_e_forward_declaration_must_be_resolved=03115_E_Deklarasi forward dari kelas "$1" harus dipecahkan di sini untuk menggunakan kelas sebagai leluhur
% Agar bisa menggunakan obyek sebagai obyek leluhur, ia harus didefinisikan
% lebih dahulu. Kesalahan ini terjadi dalam situasi berikut:
% \begin{verbatim}
%  Type ParentClas = Class;
%       ChildClass = Class(ParentClass)
%         ...
%       end;
% \end{verbatim}
% Di mana \var{ParentClass} dideklarasikan tapi tidak didefinisikan.
parser_e_no_local_operator=03116_E_Operator lokal tidak didukung
% Anda tidak bisa meng-overload secara lokal, misalnya di dalam definisi
% prosedur atau fungsi.
parser_e_proc_dir_not_allowed_in_interface=03117_E_Direktif procedure "$1" tidak dibolehkan dalam seksi interface
% Direktif prosedur ini tidak dibolehkan dalam seksi \var{interface} dari
% sebuah unit. Anda hnya bisa menggunakannya dalam seksi \var{implementation}.
parser_e_proc_dir_not_allowed_in_implementation=03118_E_Direktif procedure "$1" tidak dibolehkan dalam seksi implementation
% Direktif prosedur ini tidak didefinisikan dalam seksi \var{implementation} pada
% sebuah unit. Anda hanya bisa menggunakannya dalam seksi \var{interface}.
parser_e_proc_dir_not_allowed_in_procvar=03119_E_Direktif prosedur "$1" tidak dibolehkan dalam deklarasi procvar
% Direktif prosedur ini tidak bisa berupa bagian dari deklarasi tipe prosedural
% atau fungsi.
parser_e_function_already_declared_public_forward=03120_E_Fungsi sudah dideklarasikan Public/Forward "$1"
% Anda akan mendapat kesalahan ini jika fungsi didefiniskan sebagai \var{forward} dua kali.
% Atau sekali dalam seksi \var{interface}, dan sekali sebagai deklarasi
% \var{forward} dalam seksi \var{implmentation}.
parser_e_not_external_and_export=03121_E_Tidak bisa menggunakan EXPORT dan EXTERNAL
% Dua prosedur ini adalah mutual eksklusif
parser_n_not_supported_for_inline=03123_N_"$1" belum didukung di dalam inline procedure/function
% Inline procedures tidak mendukung deklarasi ini.
parser_h_inlining_disabled=03124_H_Inlining dimatikan
% Inlining dari prosedur dimatikan.
parser_i_writing_browser_log=03125_I_Menulis log Browser $1
% Ketika pesan informasi hidup, kompilator memperingatkan anda saat ia
% menulis log browser (dibuat dengan saklar \var{\{\$Y+ \}}).
parser_h_maybe_deref_caret_missing=03126_H_mungkin dereferensi pointer hilang
% Kompilator berpikir bahwa pointer mungkin memerlukan dereferensi.
parser_f_assembler_reader_not_supported=03127_F_Pembaca assembler yang dipilih tidak didukung
% Pembaca assembler yang dipilih (dengan \var{\{\$ASMMODE xxx\}} tidak
% didukung. Kompilator dapat dikompilasi dengan atau tanpa dukungan pembaca
% assembler tertentu.
parser_e_proc_dir_conflict=03128_E_Direktif prosedur "$1" konflik dengan direktif lain
% Anda menetapkan direktif prosedur yang konflik dengan direktif lainnya.
% Sebagai contoh \var{cdecl} dan \var{pascal} adalah mutual eksklusif.
parser_e_call_convention_dont_match_forward=03129_E_Konvensi pemanggilan forward tidak sama
% Kesalahan ini terjadi ketika anda mendeklarasikan fungsi atau prosedur dengan
% misalnya \var{cdecl;} tapi mengabaikan direktif ini dalam implementasi, atau
% sebaliknya. Konvensi pemanggilan adalah bagian dari deklarasi fungsi, dan
% harus diulang dalam definisi fungsi.
parser_e_property_cant_have_a_default_value=03131_E_Properti tidak bisa memiliki nilai default
% Set properti atau properti berindeks tidak bisa mempunyai nilai default.
parser_e_property_default_value_must_const=03132_E_Nilai default pada properti harus konstan
% Nilai dari properti yang dideklarasikan \var{default} harus dikenal waktu
% dikompilasi. nilai yang anda tetapkan hanya dikenal saat run time. Ini terjadi
% misalnya jika anda menetapkan sebuah nama variabel sebagai nilai default.
parser_e_cant_publish_that=03133_E_Simbol tidak bisa dipublikasi, hanya bisa sebuah kelas
% Hanya variabel tipe kelas yang bisa berada dalam seksi \var{published} pada
% sebuah kelas jika ia tidak dideklarasikan sebagai sebuah properti.
parser_e_cant_publish_that_property=03134_E_Jenis properti ini tidak bisa dipublikasi
% Properti dalam seksi \var{published} tidak bisa berupa properti array.
% ia harus dipindahkan ke seksi public. Properti dalam seksi \var{published}
% harus berupa tipe ordinal, tipe real, string atau set.
parser_e_empty_import_name=03136_E_Nama import diperlukan
% Beberapa target memerlukan nama untuk prosedur yang diimpor atau pembeda \var{cdecl}
parser_e_division_by_zero=03138_E_Pembagian dengan nol
% Ada pembagian dengan nol yang ditemukan
parser_e_invalid_float_operation=03139_E_Operasi floating point tidak benar
% Operasi pada dua nilai tipe real menhasilkan overflow atau pembagian dengan
% nol.
parser_e_array_lower_less_than_upper_bound=03140_E_Batas atas jangkauan kurang dari batas lebih rendah
% Batas lebih atas pada deklarasi array kurang dari batas lebih rendah dan ini 
% mungkin
parser_w_string_too_long=03141_W_string "$1" lebih panjang dari "$2"
% Ukuran konstan string lebih besar dari ukuran yang anda tetapkan dalam
% definisi tipe string
parser_e_string_larger_array=03142_E_panjang string lebih besar dari panjang array of char
% Ukuran konstan string lebih besar dari ukuran yang anda tetapkan dalam
% definisi array[x..y] of char
parser_e_ill_msg_expr=03143_E_Ekspresi tidak benar setelah direktif pesan
% \fpc hanya mendukung nilai integer atau string sebagai konstan pesan
parser_e_ill_msg_param=03144_E_Pengendali pesan hanya dapat mengambil satu panggilan dengan parameter ref.
% Metode yang dideklarasikan dengan direktif-\var{message} sebagai pengendali
% pesan hanya mengambil satu parameter yang harus dideklarasikan sebagai 
% panggilan dengan referensi. Parameter dideklarasikan sebagai panggilan dengan
% referensi menggunakan direktif-\var{var}
parser_e_duplicate_message_label=03145_E_Duplikasi label pesan: "$1"
% Label untuk sebuah pesan dipakai dua kali dalam satu object/class
parser_e_self_in_non_message_handler=03146_E_Self hanya bisa berupa parameter eksplisit dalam metode yang merupakan pengendali pesan
% Parameter self hanya dapat dioper secara eksplisit ke metode yang 
% dideklarasikan sebagai pengendali pesan.
parser_e_threadvars_only_sg=03147_E_Threadvars hanya bisa berupa static atau global
% Threadvars harus berupa static atau global, anda tidak bisa mendeklarasikan
% lokal thread ke prosedur. Variabel lokal selalu lokal bagi thread, karena
% setiap thread memiliki stack-nya sendiri dan variabel lokal disimpan pada
% stack
parser_f_direct_assembler_not_allowed=03148_F_Assembler langsung tidak didukung untuk format output biner
% Anda tidak dapat menggunakan assembler langsung saat menggunakan penulis biner,
% pilih format output lain atau gunakan pembaca assembler lain
parser_w_no_objpas_use_mode=03149_W_Jangan ambil unit OBJPAS secara manual, sebaliknya gunakan \{\$mode objfpc\} atau \{\$mode delphi\}
% Anda mencoba untuk mengambil unit ObjPas secara manual dari kausul uses. Ini
% bukan ide yang baik. Gunakan direktif \var{\{\$mode objfpc\}} atau
% \var{\{\$mode delphi\}}
% yang mengambil unit secara otomatis
parser_e_no_object_override=03150_E_OVERRIDE tidak bisa dipakai dalam obyek
% Override tidak didukung untuk obyek, sebaliknya gunakan \var{virtual} untuk 
% mengganti metode obyek leluhur
parser_e_cant_use_inittable_here=03151_E_Tipe data yang memerlukan initialization/finalization tidak bisa dipakai dalam variant record
% Beberapa tipe data (misalnya \var{ansistring}) memerlukan kode initialization/finalization
% yang secara implisit dibuat oleh kompilator. Tipe data sepert itu tidak
% bisa dipakai dalam bagian varian pada sebuah record.
parser_e_resourcestring_only_sg=03152_E_Resourcestring hanya berupa static atau global
% Resourcestring tidak bisa dideklarasikan lokal, hanya global atau menggunakan
% direktif static.
parser_e_exit_with_argument_not__possible=03153_E_Exit dengan argumen tidak bisa dipakai di sini
% Pernyataan exit dengan sebuah argumen untuk nilai balik tidak bisa dipakai
% di sini, ini terjadi misalnya dalam blok \var{try..except} atau \var{try..finally}
parser_e_stored_property_must_be_boolean=03154_E_Tipe penyimpanan simbol harus boolean
% Jika anda menetapkan penyimpanan simbol dalam deklarasi properti, ia harus 
% berupa tipe boolean
parser_e_ill_property_storage_sym=03155_E_Simbol ini tidak diijinkan sebagai penyimpanan simbol
% Anda tidak bisa menggunakan tipe ini pada simbol sebagai pembeda penyimpanan
% dalam deklarasi properti. Anda hanya bia menggunakan metode dengan tipe hasil
% boolean, field kelas boolean atau konstan boolean
parser_e_only_publishable_classes_can_be_published=03156_E_Hanya kelas yang dikompilasi dalam mode $M+ yang bisa dipublikasikan
% Dalam seksi published dari kelas hanya bisa berupa kelas sebagai field yang dipakai 
% dikompilasi dalam \var{\{\$M+\}} atau yang diturunkan dari kelas seperti itu.
% Normalnya kelas demikian harus dideklarasikan dari TPersitent
parser_e_proc_directive_expected=03157_E_Direktif prosedur diharapkan
% Kesalahan ini dipicu ketika anda mempunyai direktif \var{\{\$Calling\}} tanpa
% konvensi pemanggilan yang ditetapkan.
% Ia juga terjadi saat mendeklarasikan prosedur dalam blok const dan anda
% menggunakan sebuah ; setelah deklarasi procedure yang harus diikuti oleh
% sebuah direktif procedure.
% Deklarasi yang benar adalah:
% \begin{verbatim}
% const
%   p : procedure;stdcall=nil;
%   p : procedure stdcall=nil;
% \end{verbatim}
parser_e_invalid_property_index_value=03158_E_Nilai indeks properti harus tipe ordinal
% Nilai yang anda pakai untuk mengindeks properti harus berupa tipe ordinal, 
% sebagai contoh tipe integer atau dienumerasi.
parser_e_procname_to_short_for_export=03159_E_Nama prosedur terlalu pendek untuk diekspor
% Panjang nama procedure/function harus setidaknya 2 karakter. Ini dikarenakan
% bug dalam dlltool yang tidak menguraikan file .def dengan benar
% dengan panjang nama 1.
parser_e_dlltool_unit_var_problem=03160_E_Tidak ada entri DEFFILE dapat dibuat untuk unit global vars
parser_e_dlltool_unit_var_problem2=03161_E_Kompilasi tanpa opsi -WD
% Anda perlu mengompilasi file ini tanpa saklar -WD pada baris
% perintah
parser_f_need_objfpc_or_delphi_mode=03162_F_Anda perlu ObjFpc (-S2) atau mode Delphi (-Sd) untuk mengompilasi modul ini
% Anda perlu menggunakan \{\$mode objfpc\} atau \{\$mode delphi\} untuk
% mengompilasi file ini. Atau gunakan saklar commandline yang sama -S2 atau -Sd.
parser_e_no_export_with_index_for_target=03163_E_Tidak bisa mengekspor dengan indeks di bawah $1
% Mengekspor fungsi atau prosedur dengan indeks yang ditetapkan tidak didukung
% pada target ini.
parser_e_no_export_of_variables_for_target=03164_E_Mengekspor variabel tidak didukung di bawah $1
% Mengekspor variabel tidak didukung pada target ini.
parser_e_improper_guid_syntax=03165_E_Sintaks GUID tidak benar
% Indikasi GUID tidak memiliki sintaks yang benar. Ia seharusnya dalam bentuk
% \begin{verbatim}
% {XXXXXXXX-XXXX-XXXX-XXXX-XXXXXXXXXXXX}
% \end{verbatim}
% Di mana setiap \var{X} mewakili digit heksadesimal.
parser_w_interface_mapping_notfound=03168_W_Prosedur bernama "$1" tidak meemukan yang sesuai untuk mengimplementasikan $2.$3
% Kompilator tidak bisa menemukan prosedur yang cocok yang mengimplementasikan metode
% yang diberikan pada interface. Prosedur dengan nama sama ditemukan, tapi argumen tidak sama.
parser_e_interface_id_expected=03169_E_pengenal interface diharapkan
% Ini terjadi ketika kompilator memindai deklarasi \var{class} yang berisi
% \var{interface} nama fungsi memetakan kode seperti ini:
% \begin{verbatim}
% type
%   TMyObject = class(TObject, IDispatch)
%     function IUnknown.QueryInterface=MyQueryInterface;
%     ....
% \end{verbatim}
% dan \var{interface} sebelum titik tidak didaftarkan dalam daftar turunan.
parser_e_type_cant_be_used_in_array_index=03170_E_Tipe "$1" tidak bisa dipakai sebagai tipe indeks array
% Tipe seperti \var{qword} atau \var{int64} tidak dibolehkan sebagai tipe indeks array
parser_e_no_con_des_in_interfaces=03171_E_Con- dan destruktor tidak dibolehkan dalam interface
% Deklarasi konstruktor dan destruktor tidak dibolehkan dalam interface
% Dalam banyak kasus metode \var{QueryInterface} pada \var{IUnknown} dapat
% dipakai untuk membuat interface baru.
parser_e_no_access_specifier_in_interfaces=03172_E_Pembeda akses tidak bisa dipakai dalam INTERFACE
% Pembeda akses \var{public}, \var{private}, \var{protected} dan
% \var{pusblished} tidak bisa dipakai dalam interfaces karena semua metode
% pada interfaces harus public.
parser_e_no_vars_in_interfaces=03173_E_Interface tidak bisa berisi field
% Deklarasi field tidak dibolehkan dalam interface. Interface hanya dapat
% berisi metode
parser_e_no_local_proc_external=03174_E_Tidak bisa mendeklarasikan prosedur lokal sebagai EXTERNAL
% Mendeklarasikan prosedur lokal sebagai external tidak mungkin. Prosedur lokal
% mendapat parameter tersembunyi yang akan menjadi peluang kesalahan sangat tinggi
parser_w_skipped_fields_before=03175_W_Beberapa field datang sebelum "$1" diinisialisasi
% Dalam mode Delphi, tidak semua field dari record tipe konstan harus
% diinisialisasi, tapi kompilator memperingatkan anda saat ia mendeteksi situasi itu.
parser_e_skipped_fields_before=03176_E_Beberapa field datang sebelum "$1" diinisialisasi
% Dalam semua mode sintaks kecuali mode Delphi, anda tidak bisa meninggalkan 
% beberapa field tidak diinisialisasi di tengah konstan tipe record
parser_w_skipped_fields_after=03177_W_Beberapa field datang setelah "$1" tidak diinisialisasi
% Anda dapat meninggalkan beberapa fields di akhir konstan tipe record tidak
% diinisialisasi (kompilator akan menginisialisasinya ke nol secara otomatis).
% Ini dapat menjadi penyebab masalah yang kentara.
parser_e_varargs_need_cdecl_and_external=03178_E_Direktif VarArgs (atau '...' dalam MacPas) tanpa CDecl/CPPDecl/MWPascal dan External
% Direktif varargs (atau parameter varargs ``...'' dalam mode MacPas) hanya bisa
% dipakai dengan prosedur atau fungsi yang dideklarasikan dengan \var{external} dan salah satu 
% dari \var{cdecl}, \var{cppdecl} dan \var{mwpascal}. Fungsionalitas ini hanya
% didukung untuk menyediakan interface kompatibel terhadap fungsi C seperti printf.
parser_e_self_call_by_value=03179_E_Self harus berupa parameter normal (dipanggil-dengan-nilai)
% Anda tidak bisa mendeklarasikan self sebagai parameter const atau var, ia harus
% selalu berupa parameter dipanggil-dengan-nilai
parser_e_interface_has_no_guid=03180_E_Interface "$1" tidak mempunyai identifikasi interface
% Ketika anda ingin menempatkan interface ke konstan, maka interface harus
% sudah menyetel nilai GUID.
parser_e_illegal_field_or_method=03181_E_Field class atau pengenal metode "$1" tidak dikenal
% Properti harus merujuk ke sebuah field atau metode dalam kelas yang sama.
parser_w_proc_overriding_calling=03182_W_Mengganti konvensi pemanggilan "$1" dengan "$2"
% Ada dua direktif dalam deklarasi prosedur yang menetapkan konvensi
% pemanggilan. Hanya direktif terakhir yang akan dipakai
parser_e_no_procvarobj_const=03183_E_Tipe konstan dari tipe "procedure of object" hanya dapat diinisialisasi dengan NIL
% Anda tidak bisa menempatkan alamat sebuah metode ke tipe konstan yang
% memiliki tipe 'procedure of object', karena konstan memerlukan dua alamat:
% yang metodenya (dikenal saat waktu kompilasi) dan yang obyeknya atau turunan
% beroperasi pada kelasnya (yang tidak dapat dikenal saat waktu kompilasi).
parser_e_default_value_only_one_para=03184_E_Nilai default hanya bisa ditempatkan ke satu parameter
% Tidak mungkin untuk menetapkan nilai default untuk beberapa parameters sekaligus.
% Yang berikut adalah tidak benar:
% \begin{verbatim}
% Procedure MyProcedure (A,B : Integer = 0);
% \end{verbatim}
% Lebih baik, deklarasikan sebagai
% \begin{verbatim}
% Procedure MyProcedure (A : Integer = 0; B : Integer = 0);
% \end{verbatim}
parser_e_default_value_expected_for_para=03185_E_Parameter default diperlukan untuk "$1"
% Parameter yang ditetapkan memerlukan nilai default.
parser_w_unsupported_feature=03186_W_Penggunaan fitur yang tidak didukung!
% Anda mencoba untuk memaksa kompilator melakukan yang belum bisa dilakukannya.
parser_h_c_arrays_are_references=03187_H_Array C dikirimkan dengan referensi
% Setiap array yang dikirimkan ke fungsi C dioper dengan pointer
% (misalnya dengan referensi).
parser_e_C_array_of_const_must_be_last=03188_E_Array of const pada C harus argumen terakhir
% Anda tidak bisa menambah argumen lain setelah \var{array of const} untuk 
% fungsi \var{cdecl}, karena ukuran yang disimpan pada stack untuk argumen ini
% tidak diketahui.
parser_h_type_redef=03189_H_Redefinisi tipe "$1"
% Ini adalah indikator yang sebelumnya tipe dideklarasikan sedang didefinisikan
% ulang sebagai yang lain. Ini bisa menjadi atau tidak penyebab dari
% kesalahan.
parser_w_cdecl_has_no_high=03190_W_fungsi cdecl yang dideklarasikan tidak memiliki parameter tinggi
% Fungsi yang dideklarasikan dengan pengubah cdecl tidak mengirimkan parameter ekstra implisit.
parser_w_cdecl_no_openstring=03191_W_fungsi cdecl yang dideklarasikan tidak mendukung string terbuka
% Openstring tidak didukung untuk fungsi yang dideklarasikan cdecl.
parser_e_initialized_not_for_threadvar=03192_E_Tidak bisa menginisialisasi variabel yang dideklarasikan sebagai threadvar
% Variabel yang dideklarasikan sebagai threadvar tidak bisa diinisialisasi dengan
% nilai default. Variabel akan selalu diisi dengan nol di awal thread baru.
parser_e_msg_only_for_classes=03193_E_Direktif message hanya dibolehkan dalam Kelas
% Direktif pesan hanya didukung untuk tipe Class.
parser_e_procedure_or_function_expected=03194_E_Procedure atau Function diharapkan
% Metode kelas hanya bisa ditetapkan untuk prosedur dan fungsi.
parser_e_illegal_calling_convention=03195_W_Direktif konvensi pemanggilan diabaikan: "$1"
% Beberapa konvensi pemanggilan hanya didukung oleh CPU tertentu. Misalnya dukungan banyak non-i386 port
% hanya konvensi pemanggilan standar ABI pada CPU.
parser_e_no_object_reintroduce=03196_E_REINTRODUCE tidak bisa dipakai dalam obyek
% \var{reintroduce} tidak didukung untuk obyek.
parser_e_paraloc_only_one_para=03197_E_Setiap argumen harus memiliki lokasi sendiri
% Jika lokasi untuk argumen ditetapkan secara eksplisit karena ia diperlukan oleh
% beberapa konvensi syscall, setiap argumen harus memiliki lokasinya sendiri, hal
% seperti \var{procedure p(i,j : longint 'r1');} tidak diijinkan
parser_e_paraloc_all_paras=03198_E_Setiap argumen harus memiliki lokasi eksplisit
% Jika satu argumen mempunyai lokasi argumen eksplisit, semua argument pada 
% prosedur harus memilikinya.
parser_e_illegal_explicit_paraloc=03199_E_Lokasi argumen tidak dikenal
% Lokasi yang ditetapkan untuk argumen tidak dikenal oleh kompilator
parser_e_32bitint_or_pointer_variable_expected=03200_E_32 Bit-Integer atau variabel pointer diharapkan
% Libbase untuk MorphOS/AmigaOS hanya dapat diberikan sebagai \var{longint}, \var{dword} atau variabel pointer.
parser_e_goto_outside_proc=03201_E_Pernyataan goto tidak dibolehkan diantara prosedure yang berbeda
% Tidak diperkenankan menggunakan pernyataan \var{goto} mereferensi label di luar
% prosedur saat ini. Contoh berikut menampilkan masalah ini:
% \begin{verbatim}
% ...
%   procedure p1;
%   label
%     l1;
%
%     procedure p2;
%     begin
%       goto l1; // Goto ini TIDAK diijinkan
%     end;
%
%   begin
%     p2
%   l1:
%   end;
% ...
%
% \end{verbatim}
parser_f_too_complex_proc=03202_F_Prosedur terlalu kompleks, memerlukan terlalu banyak register
% Badan prosedur anda terlalu panjang bagi kompilator. Anda harus memisahkan
% prosedur ke dalam multipel prosedure lebih kecil.
parser_e_illegal_expression=03203_E_Ekspresi tidak benar
% Ini dapat terjadi di bawah banyak keadaan. Terutama saat mencoba untuk
% mengevaluasi ekspresi konstan.
parser_e_invalid_integer=03204_E_Ekspresi integer tidak benar
% Anda membuat ekspresi yang bukan integer, dan kompilator mengharapkan hasilnya
% adalah integer.
parser_e_invalid_qualifier=03205_E_Kualifier tidak benar
% Salah satu dari yang berikut terjadi :
% \begin{itemize}
% \item Anda mencoba untuk mengakses field variabel yang bukan sebuah record.
% \item Anda mengindeks variabel yang bukan sebuah array.
% \item Anda medereferensi variabel yang bukan sebuah pointer.
% \end{itemize}
parser_e_upper_lower_than_lower=03206_E_Batas tinggi jakauan < batas rendah jangkauan
% You are declaring a subrange, and the lower limit is higher than the high
% limit of the range.
parser_e_macpas_exit_wrong_param=03207_E_Parameter Exit harus berupa nama prosedur yang dipakai
% Exit non lokal tidak dibolehkan. Kesalahan ini hanya terjadi dalam mode MacPas.
parser_e_illegal_assignment_to_count_var=03208_E_Penempatan ke variabel for-loop "$1" tidak benar
% Tipe variabel \var{for} loop harus berupa tipe ordinal.
% Variabel loop tidak bisa real atau string. Anda juga tidak bisa menempatkan
% nilai ke variabel loop di dalam loop (kecuali dalam mode Delphi dan TP). 
% Sebaliknya gunakan while atau repeat loop jika anda perlu melakukan sesuatu
% seperti itu, karena konstruksinya dibangun untuk itu.
parser_e_no_local_var_external=03209_E_Tidak bisa mendeklarasikan variabel lokal sebagai EXTERNAL
% Mendeklarasikan variabel lokal sebagai external tidak dibolehkan. Hanya
% variabel global dapat mereferensi ke variabel external.
parser_e_proc_already_external=03210_E_Procedure sudah dideklarasikan EXTERNAL
% Prosedur sudah dideklarasikan dengan direktif EXTERNAL dalam interface atau
% deklarasi forward.
parser_w_implicit_uses_of_variants_unit=03211_W_Penggunaan implisit unit Variants
% Tipe Variant dipakai dalam unit tanpa menggunakan unit Variants. Kompilator
% telah menambahkan secara implisit unit Variants ke akhir daftar uses. Untuk
% menghapus peringatan ini, unit Variants perlu ditambahkan ke pernyataan uses.
parser_e_no_static_method_in_interfaces=03212_E_Kelas dan metode statis tidak bisa dipakai dalam INTERFACE
% Pembeda \var{class} dan direktif \var{static} tidak bisa dipakai dalam interface
% karena semua metode interface harus public.
parser_e_arithmetic_operation_overflow=03213_E_Overflow dalam operasi aritmatika
% Operasi pada dua nilai integer menghasilkan overflow
parser_e_protected_or_private_expected=03214_E_Protected atau private diharapkan
% \var{strict} hanya bisa dipakai bersama dengan \var{protected} atau \var{private}.
parser_e_illegal_slice=03215_E_SLICE tidak bisa dipakai di luar daftar parameter
% \var{slice} hanya bisa dipakai untuk argumen yang menerima parameter array terbuka
parser_e_dispinterface_cant_have_parent=03216_E_DISPINTERFACE tidak bisa mempunyai kelas leluhur
% DISPINMTERFACE adalah tipe khusus interface yang tidak memiliki kelas leluhur
parser_e_dispinterface_needs_a_guid=03217_E_DISPINTERFACE memerlukan sebuah guid
% DISPINMTERFACE selalu memerlukan identifikasi interface
parser_w_overridden_methods_not_same_ret=03218_W_Metode yang diganti harus tipe balik terkait. Kode ini mungkin rusak, ia tergantung pada bug pengurai Delphi ("$2" diganti dengan "$1" yang mempunyai tipe balik lain)
% Jika anda mendeklarasikan metode pengganti dalam definisi kelas, ia harus
% mempunyai tipe balik yang sama. Beberapa versi Delphi membolehkan anda untuk
% mengubah tipe balik pada metode interface, dan bahkan mengubah prosedur menjadi
% fungsi, tapi kode hasil bisa rusak tergantung pada tipe yang dipakai dan
% cara metode dipanggil.
parser_e_dispid_must_be_ord_const=03219_E_Dispatch ID harus konstan ordinal
% The \var{dispid} keyword must be followed by an ordinal constant (the dispid index).
parser_e_array_range_out_of_bounds=03220_E_Jangkauan array terlalu besar
% Meskipun ukuran tergantung elemennya, array tidak bisa memiliki lebih dari
% high(ptrint) elemen. Sebagai tambahan, tipe jangkauan harus sub jangkauan
% dari ptrint.
parser_e_packed_element_no_var_addr=03221_E_Alamat tidak bisa diambil dari elemen packed array dan field record
% Jika anda mendeklarasikan array atau record sebagai \var{packed} dalam mode Mac Pascal 
% (atau sebagai \var{packed} dalam setiap mode dengan \var{\{\$bitpacking on\}}),
% ia akan di-packed di tingkat bit. Ini berarti ia menjadi tidak mungkin untuk 
% mengambil alamat dari elemen individual array atau field record. Ini hanya 
% kekecualian pada aturan ini bila elemen packed array yang ukuran pemadatannya
% adalah multpele dari 8 bit.
parser_e_packed_dynamic_open_array=03222_E_Array dinamis tidak bisa dipadatkan
% Hanya array reguler (dan mungkin ke depan juga yang terbuka) dapat dipadatkan
parser_e_packed_element_no_loop=03223_E_Bit elemen packed array dan field record tidak bisa dipakai sebagai variabel loop
% Jika anda mendeklarasikan array atau record sebagai \var{packed} dalam mode Mac Pascal (atau sebagai \var{packed} dalam setiap mode \var{\{\$bitpacking on\}}),
% ia akan dipadatkan di tingkat bit. Untuk alasan performansi, ia tidak bisa
% dipakai sebagai variabel loop.
parser_e_type_var_const_only_in_records_and_classes=03224_E_VAR, TYPE and CONST are allowed only in records, objects and classes
% The usage of VAR, TYPE and CONST to declare new types inside an object is allowed only inside
% records, objects and classes.
parser_e_cant_create_generics_of_this_type=03225_E_Tipe ini tidak bisa generik
% Hanya Class, Object, Interface dan Record dibolehkan untuk dipakai sebagai generik
parser_w_no_lineinfo_use_switch=03226_W_Jangan ambil unit LINEINFO secara manual, sebaliknya gunakan saklar kompilator -gl
% Jangan menggunakan unit LINEINFO secara langsung, gunakan saklar \var{-gl}
% yang secara otomatis menambahkan unit untuk membaca tipe informasi debug.
parser_e_no_funcret_specified=03227_E_Tidak ada tipe hasil fungsi yang ditetapkan untuk "$1"
% Pertama kali anda mendeklarasikan fungsi anda harus mendeklarasikannya dengan lengkap,
% termasuk seluruh parameter dan tipe hasil.
parser_e_special_onlygenerics=03228_E_Specialization hanya didukung untuk tipe generik
% Tipe yang bukan generik tidak bisa di spesialisasi
parser_e_no_generics_as_params=03229_E_Generik tidak bisa dipakai sebagai parameter saat menspesialisasi generik
% Ketika menspesialisasi generik, hanya tipe non-generik yang dapat dipakai sebagai parameter.
parser_e_type_object_constants=03230_E_Konstan obyek yang berisi VMT tidak dibolehkan
% Jika sebuah obyek memerlukan VMT baik karena ia berisi konstruktor ataupun metode virtual,
% tidak diijinkan untuk membuat konstan darinya. Dalam mode TP dan Delphi ini 
% dibolehkan untuk alasan kompatibilitas.
parser_e_label_outside_proc=03231_E_Mengambil alamat label yang didefinisikan di luar lingkup saat ini tidak dibolehkan
% Tidak diperbolehkan untuk mengambil alamat label di luar prosedur
% saat ini.
parser_e_initialized_not_for_external=03233_E_Tidak bisa menginisialisasi variabel yang dideklarasikan sebagai external
% Variabel yang dideklarasikan sebagai external tidak bisa diinisialisasi dengan nilai default.
parser_e_illegal_function_result=03234_E_Tipe hasil fungsi tidak benar
% Beberapa tipe seperti tipe file tidak bisa dipakai sebagai hasil fungsi.
parser_e_no_common_type=03235_E_Tidak ada tipe umum yang mungkin antara "$1" dan "$2"
% Untuk melakukan operasi diantara integer, kompilator mengubah kedua operand
% ke tipe umum, yang terlihat tipe tidak benar. Untuk menentukan tipe umum
% dari operand, kompilator mengambil minimum dari nilai minimal kedua tipe,
% dan maksimum dari maksimal nilai kedua tipe. Tipe umum adalah 
% minimum..maksimum.
parser_e_no_generics_as_types=03236_E_Generik tanpa spesialisasi tidak bisa dipakai sebagai tipe untuk variabel
% Generik harus selalu dispesialisasi sebelum dipakai sebagai tipe variabel
% \end{description}
#
# Pemeriksaan Tipe
#
# 04082 is the last used one
#
% \section{Kesalahan pemeriksaan tipe}
% Seksi ini mendaftarkan semua kesalahan yang terjadi saat pemeriksaan tipe
% dilakukan.
% \begin{description}
parser_w_register_list_ignored=03237_W_Register list is ignored for pure assembler routines
% When using pure assembler routines, the list with modified registers is ignored.
parser_e_implements_must_be_class_or_interface=03238_E_Implements property must have class or interface type
% A property which implements an interface must be of type class or interface.
parser_e_implements_must_have_correct_type=03239_E_Implements-property must implement interface of correct type, found "$1" expected "$2"
% A property which implements an interface actually implements a different interface.
parser_e_implements_must_read_specifier=03240_E_Implements-property must have read specifier
% A property  which implements an interface must have at least a read specifier.
parser_e_implements_must_not_have_write_specifier=03241_E_Implements-property must not have write-specifier
% A property  which implements an interface may not have a write specifier.
parser_e_implements_must_not_have_stored_specifier=03242_E_Implements-property must not have stored-specifier
% A property  which implements an interface may not have a stored specifier.
parser_e_implements_uses_non_implemented_interface=03243_E_Implements-property used on unimplemented interface: "$1"
% The interface which is implemented by a property is not an interface implemented by the class.
parser_e_unsupported_real=03244_E_Floating point not supported for this target
% The compiler parsed a floating point expression, but it is not supported.
parser_e_class_doesnt_implement_interface=03245_E_Class "$1" does not implement interface "$2"
% The delegated interface is not implemented by the class given in the implements clause.
parser_e_class_implements_must_be_interface=03246_E_Type used by implements must be an interface
% The \var{implements} keyword must be followed by an interface type.
parser_e_cant_export_var_different_name=03247_E_Variables cannot be exported with a different name on this target, add the name to the declaration using the "export" directive (variable name: $1, declared export name: $2)
% On most targets it is not possible to change the name under which a variable
% is exported inside the \var{exports} statement of a library.
% In that case, you have to specify the export name at the point where the
% variable is declared, using the \var{export} and \var{alias} directives.
parser_e_weak_external_not_supported=03248_E_Weak external symbols are not supported for the current target
% A "weak external" symbol is a symbol which may or may not exist at (either static
% or dynamic) link time. This concept may not be available (or implemented yet)
% on the current cpu/OS target.
parser_e_forward_mismatch=03249_E_Forward type definition does not match
% Classes and interfaces being defined forward must have the same type
% when being implemented. A forward interface cannot be changed into a class.
parser_n_ignore_lower_visibility=03250_N_Virtual method "$1" has a lower visibility ($2) than parent class $3 ($4)
% The virtual method overrides an method that is declared with a higher visibility. This might give
% unexpected results. E.g., in case the new visibility is private then a call to ``inherited'' in a
% new child class will call the higher-visible method in a parent class and ignores the private method.
parser_e_field_not_allowed_here=03251_E_Fields cannot appear after a method or property definition, start a new visibility section first
% Once a method or property has been defined in a class or object, you cannot define any fields afterwards
% without starting a new visibility section (such as \var{public}, \var{private}, etc.). The reason is
% that otherwise the source code can appear ambiguous to the compiler, since it is possible to use modifiers
% such as \var{default} and \var{register} also as field names.
parser_e_no_local_para_def=03252_E_Parameters or result types cannot contain local type definitions. Use a separate type definition in a type block.
% In Pascal, types are not considered to be identical simply because they are semantically equivalent.
% Two variables or parameters are only considered to be of the same type if they refer to the
% same type definition.
% As a result, it is not allowed to define new types inside parameter lists, because then it is impossible to
% refer to the same type definition in the procedure headers of the interface and implementation of a unit
% (both procedure headers would define a separate type). Keep in mind that expressions such as
% ``file of byte'' or ``string[50]'' also define a new type.
parser_e_abstract_and_sealed_conflict=03253_E_ABSTRACT and SEALED conflict
% ABSTRACT and SEALED cannot be used together in one declaration
parser_e_sealed_descendant=03254_E_Cannot create a descendant of the sealed class "$1"
% Sealed means that class cannot be derived by another class.
parser_e_sealed_class_cannot_have_abstract_methods=03255_E_SEALED class cannot have an ABSTRACT method
% Sealed means that class cannot be derived. Therefore no one class is able to override an abstract method in a sealed class.
parser_e_only_virtual_methods_final=03256_E_Only virtual methods can be final
% You are declaring a method as final, when it is not declared to be
% virtual.
parser_e_final_can_no_be_overridden=03257_E_Final method cannot be overridden: "$1"
% You are trying to \var{override} a virtual method of a parent class that does
% not exist.
parser_e_multiple_messages=03258_E_Only one message can be used per method.
% It is not possible to associate multiple messages with a single method.
parser_e_invalid_enumerator_identifier=03259_E_Invalid enumerator identifier: "$1"
% Only "MoveNext" and "Current" enumerator identifiers are supported.
parser_e_enumerator_identifier_required=03260_E_Enumerator identifier required
% "MoveNext" or "Current" identifier must follow the \var{enumerator} modifier.
parser_e_enumerator_movenext_is_not_valid=03261_E_Enumerator MoveNext pattern method is not valid. Method must be a function with the Boolean return type and no required arguments.
% "MoveNext" enumerator pattern method must be a function with Boolean return type and no required arguments
parser_e_enumerator_current_is_not_valid=03262_E_Enumerator Current pattern property is not valid. Property must have a getter.
% "Current" enumerator pattern property must have a getter
parser_e_only_one_enumerator_movenext=03263_E_Only one enumerator MoveNext method is allowed per class/object
% Class or Object can have only one enumerator MoveNext declaration.
parser_e_only_one_enumerator_current=03264_E_Only one enumerator Current property is allowed per class/object
% Class or Object can have only one enumerator Current declaration.
parser_e_for_in_loop_cannot_be_used_for_the_type=03265_E_For in loop cannot be used for the type "$1"
% For in loop can be used not for all types. For example it cannot be used for the enumerations with jumps.
parser_e_objc_requires_msgstr=03266_E_Objective-C messages require their Objective-C selector name to be specified using the "message" directive.
% Objective-C messages require their Objective-C name (selector name) to be specified using the \var{message `someName:'} procedure directive.
% While bindings to other languages automatically generate such names based on the identifier you use (by replacing
% all underscores with colons), this is unsafe since nothing prevents an Objective-C method name to contain actual
% colons.
parser_e_objc_no_constructor_destructor=03267_E_Objective-C does not have formal constructors nor destructors. Use the alloc, initXXX and dealloc messages.
% The Objective-C language does not have any constructors or destructors. While there are some messages with a similar
% purpose (such as \var{init} and \var{dealloc}), these cannot be identified using automatic parsers and do not
% guarantee anything like Pascal constructors/destructors (e.g., you have to take care of only calling ``designated''
% inherited ``constructors''). For these reasons, we have opted to follow the standard Objective-C patterns for
% instance creation/destruction.
parser_e_message_string_too_long=03268_E_Message name is too long (max. 255 characters)
% Due to compiler implementation reasons, message names are currently limited to 255 characters.
parser_e_objc_message_name_too_long=03269_E_Objective-C message symbol name for "$1" is too long
% Due to compiler implementation reasons, mangled message names (i.e., the symbol names used in the assembler
% code) are currently limited to 255 characters.
parser_h_no_objc_parent=03270_H_Defining a new Objective-C root class. To derive from another root class (e.g., NSObject), specify it as the parent class.
% If no parent class is specified for an Object Pascal class, then it automatically derives from TObject.
% Objective-C classes however do not automatically derive from NSObject, because one can have multiple
% root classes in Objective-C. For example, in the Cocoa framework both NSObject and NSProxy are root classes.
% Therefore, you have to explicitly define a parent class (such as NSObject) if you want to derive your
% Objective-C class from it.
parser_e_no_objc_published=03271_E_Objective-C classes cannot have published sections.
% In Object Pascal, ``published'' determines whether or not RTTI is generated. Since the Objective-C runtime always needs
% RTTI for everything, this specified does not make sense for Objective-C classes.
parser_f_need_objc=03272_F_This module requires an Objective-C mode switch to be compiled
% This error indicates the use of Objective-C language features without an Objective-C mode switch
% active. Enable one via the -M command line switch, or the {\$modeswitch x} directive.
parser_e_must_use_override=03273_E_Inherited methods can only be overridden in Objective-C and Java, add "override" (inherited method defined in $1)
parser_h_should_use_override=03274_H_Inherited methods can only be overridden in Objective-C and Java, add "override" (inherited method defined in $1).
% It is not possible to \var{reintroduce} methods in Objective-C or Java like in Object Pascal. Methods with the same
% name always map to the same virtual method entry. In order to make this clear in the source code,
% the compiler always requires the \var{override} directive to be specified when implementing overriding
% Objective-C or Java methods in Pascal. If the implementation is external, this rule is relaxed because Objective-C and Java
% do not have any \var{override}-style keyword (since it's the default and only behaviour in these languages),
% which makes it hard for automated header conversion tools to include it everywhere.
% The type in which the inherited method is defined is explicitly mentioned, because this may either
% be an objcclass or an objccategory in case of Objective-C.
parser_e_objc_message_name_changed=03275_E_Message name "$1" in inherited class is different from message name "$2" in current class.
% An overriding Objective-C method cannot have a different message name than an inherited method. The reason
% is that these message names uniquely define the message to the Objective-C runtime, which means that
% giving them a different message name breaks the ``override'' semantics.
parser_e_unique_unsupported=03276_E_It is not yet possible to make unique copies of Objective-C or Java types
% Duplicating an Objective-C or Java type using \var{type x = type y;} is not yet supported. You may be able to
% obtain the desired effect using \var{type x = objcclass(y) end;} resp.{} \var{type x = class(y) end;} instead.
parser_e_no_category_as_types=03277_E_Objective-C categories and Object Pascal class helpers cannot be used as types
% It is not possible to declare a variable as an instance of an Objective-C
% category or an Object Pascal class helper. A category/class helper adds
% methods to the scope of an existing class, but does not define a type by
% itself. An exception of this rule is when inheriting an Object Pascal class
% helper from another class helper.
parser_e_no_category_override=03278_E_Categories do not override, but replace methods. Use "reintroduce" instead.
parser_e_must_use_reintroduce_objc=03279_E_Replaced methods can only be reintroduced in Objective-C, add "reintroduce" (replaced method defined in $1).
parser_h_should_use_reintroduce_objc=03280_H_Replaced methods can only be reintroduced in Objective-C, add "reintroduce" (replaced method defined in $1).
% A category replaces an existing method in an Objective-C class, rather than that it overrides it.
% Calling an inherited method from an category method will call that method in
% the extended class' parent, not in the extended class itself. The
% replaced method in the original class is basically lost, and can no longer be
% called or referred to. This behaviour corresponds somewhat more closely to
% \var{reintroduce} than to \var{override} (although in case of \var{reintroduce}
% in Object Pascal, hidden methods are still reachable via inherited).
% The type in which the inherited method is defined is explicitly mentioned, because this may either
% be an objcclass or an objccategory.
parser_e_implements_getter_not_default_cc=03281_E_Getter for implements interface must use the target's default calling convention.
% Interface getters are called via a helper in the run time library, and hence
% have to use the default calling convention for the target (\var{register} on
% i386 and x86\_64, \var{stdcall} on other architectures).
parser_e_no_refcounted_typed_file=03282_E_Typed files cannot contain reference-counted types.
% The data in a typed file cannot be of a reference counted type (such as
% \var{ansistring} or a record containing a field that is reference counted).
parser_e_operator_not_overloaded_2=03283_E_Operator is not overloaded: $2 "$1"
% You are trying to use an overloaded operator when it is not overloaded for
% this type.
parser_e_operator_not_overloaded_3=03284_E_Operator is not overloaded: "$1" $2 "$3"
% You are trying to use an overloaded operator when it is not overloaded for
% this type.
parser_e_more_array_elements_expected=03285_E_Expected another $1 array elements
% When declaring a typed constant array, you provided to few elements to initialize the array
parser_e_string_const_too_long=03286_E_String constant too long while ansistrings are disabled
% Only when a piece of code is compiled with ansistrings enabled (\var{\{\$H+\}}), string constants
% longer than 255 characters are allowed.
parser_e_invalid_univ_para=03287_E_Type cannot be used as univ parameter because its size is unknown at compile time: "$1"
% \var{univ} parameters are compatible with all values of the same size, but this
% cannot be checked in case a parameter's size is unknown at compile time.
parser_e_only_one_class_constructor_allowed=03288_E_Only one class constructor can be declared in class: "$1"
% You are trying to declare more than one class constructor but only one class constructor can be declared.
parser_e_only_one_class_destructor_allowed=03289_E_Only one class destructor can be declared in class: "$1"
% You are trying to declare more than one class destructor but only one class destructor can be declared.
parser_e_no_paras_for_class_constructor=03290_E_Class constructors cannot have parameters
% You are declaring a class constructor with a parameter list. Class constructor methods
% cannot have parameters.
parser_e_no_paras_for_class_destructor=03291_E_Class destructors cannot have parameters
% You are declaring a class destructor with a parameter list. Class destructor methods
% cannot have parameters.
parser_f_modeswitch_objc_required=03292_F_This construct requires the \{\$modeswitch objectivec1\} mode switch to be active
% Objective-Pascal constructs are not supported when \{\$modeswitch ObjectiveC1\}
% is not active.
parser_e_widestring_to_ansi_compile_time=03293_E_Unicodechar/string constants cannot be converted to ansi/shortstring at compile-time
% It is not possible to use unicodechar and unicodestring constants in
% constant expressions that have to be converted into an ansistring or shortstring
% at compile time, for example inside typed constants. The reason is that the
% compiler cannot know what the actual ansi encoding will be at run time.
parser_e_objc_enumerator_2_0=03294_E_For-in Objective-Pascal loops require \{\$modeswitch ObjectiveC2\} to be active
% Objective-C ``fast enumeration'' support was added in Objective-C 2.0, and
% hence the appropriate modeswitch has to be activated to expose this feature.
% Note that Objective-C 2.0 programs require Mac OS X 10.5 or later.
parser_e_objc_missing_enumeration_defs=03295_E_The compiler cannot find the NSFastEnumerationProtocol or NSFastEnumerationState type in the CocoaAll unit
% Objective-C for-in loops (fast enumeration) require that the compiler can
% find a unit called CocoaAll that contains definitions for the
% NSFastEnumerationProtocol and NSFastEnumerationState types. If you get this
% error, most likely the compiler is finding and loading an alternate CocoaAll
% unit.
parser_e_no_procvarnested_const=03296_E_Typed constants of the type 'procedure is nested' can only be initialized with NIL and global procedures/functions
% A nested procedural variable consists of two components: the address of the
% procedure/function to call (which is always known at compile time), and also
% a parent frame pointer (which is never known at compile time) in case the
% procedural variable contains a reference to a nested procedure/function.
% Therefore such typed constants can only be initialized with global
% functions/procedures since these do not require a parent frame pointer.
parser_f_no_generic_inside_generic=03297_F_Declaration of generic class inside another generic class is not allowed
% At the moment, scanner supports recording of only one token buffer at the time
% (guarded by internal error 200511173 in tscannerfile.startrecordtokens).
% Since generics are implemented by recording tokens, it is not possible to
% have declaration of generic class inside another generic class.
parser_e_forward_intf_declaration_must_be_resolved=03298_E_Forward declaration "$1" must be resolved before a class can conform to or implement it
% An Objective-C protocol or Java Interface must be fully defined before classes can conform to it.
% This error occurs in the following situation (example for Objective-C, but the same goes for Java interfaces):
% \begin{verbatim}
%  Type MyProtocol = objcprotoocl;
%       ChildClass = Class(NSObject,MyProtocol)
%         ...
%       end;
% \end{verbatim}
% where \var{MyProtocol} is declared but not defined.
parser_e_no_record_published=03299_E_Record types cannot have published sections
% Published sections can be used only inside classes.
parser_e_no_destructor_in_records=03300_E_Destructors are not allowed in records or helpers
% Destructor declarations are not allowed in records or helpers.
parser_e_class_methods_only_static_in_records=03301_E_Class methods must be static in records
% Class methods declarations are not allowed in records without static modifier.
% Records have no inheritance and therefore non static class methods have no sense for them.
parser_e_no_parameterless_constructor_in_records=03302_E_Parameterless constructors are not allowed in records or record/type helpers
% Constructor declarations with no arguments are not allowed in records or record/type helpers.
parser_e_at_least_one_argument_must_be_of_type=03303_E_Either the result or at least one parameter must be of type "$1"
% It is required that either the result of the routine or at least one of its parameters be of the specified type.
% For example class operators either take an instance of the structured type in which they are defined, or they return one.
parser_e_cant_use_type_parameters_here=03304_E_Type parameters may require initialization/finalization - cannot be used in variant records
% Type parameters may be specialized with types which (e.g. \var{ansistring}) need initialization/finalization
% code which is implicitly generated by the compiler.
parser_e_externals_no_section=03305_E_Variables being declared as external cannot be in a custom section
% A section directive is not valid for variables being declared as external.
parser_e_section_no_locals=03306_E_Non-static and non-global variables cannot have a section directive
% A variable placed in a custom section is always statically allocated so it must be either a static or global variable.
parser_e_not_allowed_in_helper=03307_E_"$1" is not allowed in helper types
% Some directives and specifiers like "virtual", "dynamic", "override" are not
% allowed inside helper types in mode ObjFPC (they are ignored in mode Delphi),
% because they have no meaning within helpers. Also "abstract" isn't allowed in
% either mode.
parser_e_no_class_constructor_in_helpers=03308_E_Class constructors are not allowed in helpers
% Class constructor declarations are not allowed in helpers.
parser_e_inherited_not_in_record=03309_E_The use of "inherited" is not allowed in a record
% As records don't support inheritance the use of "inherited" is prohibited for
% these as well as for record helpers (in mode "Delphi" only).
parser_e_no_types_in_local_anonymous_records=03310_E_Type declarations are not allowed in local or anonymous records
% Records with types must be defined globally. Types cannot be defined inside records which are defined in a
% procedure or function or in anonymous records.
parser_e_duplicate_implements_clause=03311_E_Duplicate implements clause for interface "$1"
% A class may delegate an interface using the "implements" clause only to a single property. Delegating it multiple times
% is a error.
parser_e_mapping_no_implements=03312_E_Interface "$1" cannot be delegated by "$2", it already has method resolutions
% Method resolution clause maps a method of an interface to a method of the current class. Therefore the current class
% has to implement the interface directly. Delegation is not possible.
parser_e_implements_no_mapping=03313_E_Interface "$1" cannot have method resolutions, "$2" already delegates it
% Method resolution is only possible for interfaces that are implemented directly, not by delegation.
parser_e_invalid_codepage=03314_E_Invalid codepage
% When declaring a string with a given codepage, the range of valid codepages values is limited
% to 0 to 65535.
parser_e_final_only_const_var=03315_E_Only fields (var-sections) and constants can be final in object types
% A final (class) field must be assigned a single value in the (class) constructor, and cannot
% be overwritten afterwards. A final (typed) constant is read-only.
parser_e_final_only_external=03316_E_Final fields are currently only supported for external classes
% Support for final fields in non-external classes requires a full data flow
% analysis implementation in FPC, which it currently still lacks.
parser_e_no_typed_const=03317_E_Typed constants are not allowed here, only formal constants are
% Java interfaces define a namespace in which formal constant can be defined,
% but since they define no storage it is not possible to define typed constants
% in them (those are more or less the same as initialised class fields).
parser_e_java_no_inherited_constructor=03318_E_Constructors are not automatically inherited in the JVM; explicitly add a constructor that calls the inherited one if you need it
% Java does not automatically add inherited constructors to child classes, so that they can be hidden.
% For compatibility with external Java code, FPC does the same. If you require access to the same
% constructors in a child class, define them in the child class and call the inherited one from
% there.
parser_d_internal_parser_string=03319_D_Parsing internally generated code: $1
% The compiler sometimes internally constructs Pascal code that is subsequently
% injected into the program. These messages display such code, in order to help
% with debugging errors in them.
parser_e_feature_unsupported_for_vm=03320_E_This language feature is not supported on managed VM targets
% Certain language features are not supported on targets that are managed virtual machines.
parser_e_jvm_invalid_virtual_constructor_call=03321_E_Calling a virtual constructor for the current instance inside another constructor is not possible on the JVM target
% The JVM does not natively support virtual constructor. Unforunately, we are not aware of a way to
% emulate them in a way that makes it possible to support calling virtual constructors
% for the current instance inside another constructor.
parser_e_method_lower_visibility=03322_E_Overriding method "$1" cannot have a lower visibility ($2) than in parent class $3 ($4)
% The JVM does not allow lowering the visibility of an overriding method.
parser_e_nostackframe_without_assembler=03323_E_Procedure/Function declared with call option NOSTACKFRAME but without ASSEMBLER
% nostackframe call modifier is supposed to be used in conjunction with assembler.
parser_e_nostackframe_with_locals=03324_E_Procedure/Function declared with call option NOSTACKFRAME but local stack size is $1
% nostackframe call modifier used without assembler modifier
% might still generate local stack needs.
parser_e_cannot_generate_property_getter_setter=03325_E_Cannot generate property getter/setter $1 because its name clashes with existing identifier $2
% Automatically generated getters/setters cannot have the same name as existing
% identifiers, because this may change the behaviour of existing code.
parser_w_overriding_property_getter_setter=03326_W_Automatically generated property getter/setter $1 overrides the same-named getter/setter in class $2
% Automatically generated property getters/setters on the JVM platform are virtual methods, because
% the JVM does not support non-virtual methods that can be changed in child classes. This means
% that if a child class changes an inherited property definition, the behaviour of that property
% can change compared to native targets since even if a variable is declared as the parent type,
% by calling the virtual method the getter from the child will be used. This is different from
% the behaviour on native targets or when not activating automatically generated setters/getters,
% because in that case only the declared type of a variable influences the property behaviour.
parser_w_case_difference_auto_property_getter_setter_prefix=03327_W_Case mismatch between declared property getter/setter $1 and automatically constructed name $2, not changing declared name
% If a property's specified getter/setter already corresponded to the naming convention specified
% by the automatic getter/setter generation setting except in terms of upper/lowercase, the
% compiler will print a warning because it cannot necessarily change that other declaration itself
% not can it add one using the correct case (it could conflict with the original declaration).
% Manually correct the case of the getter/setter to conform to the desired coding rules.
% \var{TChild} overrides
parser_e_no_consts_in_local_anonymous_records=03328_E_Constants declarations are not allowed in local or anonymous records
% Records with constants must be defined globally. Constants cannot be defined inside records which are defined in a
% procedure or function or in anonymous records.
parser_e_no_methods_in_local_anonymous_records=03329_E_Method declarations are not allowed in local or anonymous records
% Records with methods must be defined globally. Methods cannot be defined inside records which are defined in a
% procedure or function or in anonymous records.
parser_e_no_properties_in_local_anonymous_records=03330_E_Property declarations are not allowed in local or anonymous records
% Records with properties must be defined globally. Properties cannot be defined inside records which are defined in a
% procedure or function or in anonymous records.
parser_e_no_class_in_local_anonymous_records=03331_E_Class member declarations are not allowed in local or anonymous records
% Records with class members must be defined globally. Class members cannot be defined inside records which are defined in a
% procedure or function or in anonymous records.
parser_e_not_allowed_in_record=03332_E_Visibility section "$1" not allowed in records
% The visibility sections \var(protected) and \var(strict protected) are only
% useful together with inheritance. Since records do not support that they are
% forbidden.
parser_e_dir_not_allowed=03333_E_Directive "$1" not allowed here
% This directive is not allowed in the given context. E.g. "static"
% is not allowed for instance methods or class operators.
parser_e_no_assembler_in_generic=03334_E_Assembler blocks not allowed inside generics
% The use of assembler blocks/routines is not allowed inside generics.
parser_e_property_only_sgr=03335_E_Properties can be only static, global or inside structured types
% Properties cannot be declared local, only global, using the static
% directive or inside structured types.
parser_e_overloaded_have_same_mangled_name=03336_E_Overloaded routines have the same mangled name
% Some platforms, such as the JVM platform, encode the parameters in the routine name in
% a prescribed way, and this encoding may map different Pascal types to the same encoded
% (a.k.a.\ ``mangled'') name. This error can only be solved by removing or changing the
% conflicting definitions' parameter declarations or routine names.
parser_e_default_value_val_const=03337_E_Default values can only be specified for value, const and constref parameters
% A default parameter value allows you to not specify a value for this parameter
% when calling the routine, and the compiler will instead pass the specified
% default (constant) value. As a result, default values can only be specified
% for parameters that can accept constant values.
parser_w_ptr_type_ignored=03338_W_Pointer type "$1" ignored
% The specified pointer type modifier is ignored, because it is not supported on
% the current platform. This happens, for example, when a far pointer is
% declared on a non-x86 platform.
parser_e_global_generic_references_static=03339_E_Global Generic template references static symtable
% A generic declared in the interface section of a unit must not reference symbols that belong
% solely to the implementation section of that unit.
parser_u_already_compiled=03340_UL_Unit $1 has been already compiled meanwhile.
% This tells you that the recursive reading of the uses clauses triggered already
% a compilation of the current unit, so the current compilation can be aborted.
%
%
%
% \end{description}
%
# Type Checking
#
# 04123 is the last used one
#
% \section{Type checking errors}
% This section lists all errors that can occur when type checking is
% performed.
% \begin{description}
type_e_mismatch=04000_E_Tipe tidak sama
% Ini terjadi dalam banyak kasus:
% \begin{itemize}
% \item Variabel yang anda tempatkan adalah tipe berbeda dari ekspresi dalam
% penempatan.
% \item Anda memanggil fungsi atau prosedur dengan parameters yang tidak
% kompatibel dengan parameter dalam definisi fungsi atau prosedur.
% \end{itemize}
type_e_incompatible_types=04001_E_Tipe tidak kompatibel: didapat "$1" diharapkan "$2"
% Tidak ada konversi yang mungkin diantara kedua tipe
% Kemungkinan lain adalah bahwa keduanya dideklarasikan dalam deklarasi yang
% berbeda:
% \begin{verbatim}
% Var
%    A1 : Array[1..10] Of Integer;
%    A2 : Array[1..10] Of Integer;
%
% Begin
%    A1:=A2; { Pernyataan ini juga menghasilkan kesalahan, ini
%              dikarenakan pemeriksaan tipe ketat pada pascal }
% End.
% \end{verbatim}
type_e_not_equal_types=04002_E_Tipe tidak sama antara "$1" dan "$2"
% Tipe tidak persis sama
type_e_type_id_expected=04003_E_Pengenal tipe diharapkan
% Pengenal bukan sebuah tipe, atau anda lupa menyertakan pengenal tipe.
type_e_variable_id_expected=04004_E_Pengenal variabel diharapkan
% Ini terjadi saat anda mengirimkan konstan ke sebuah rutin (seperti \var{Inc} var atau \var{Dec})
% ketia ia mengharapkan sebuah variabel. Anda juga bisa mengirimkan variabel 
% sebagai argumen ke fungsi ini.
type_e_integer_expr_expected=04005_E_Ekspresi integer diharapkan, tapi didapat "$1"
% Kompilator mengharapkan ekspresi tipe integer, tetapi mendapatkan tipe yang
% berbeda.
type_e_boolean_expr_expected=04006_E_Ekspresi boolean diharapkan, tapi didapat "$1"
% Ekspresi harus tipe boolean, ia harus mengembalikan true atau false.
type_e_ordinal_expr_expected=04007_E_Ekspresi Ordinal diharapkan
% Ekspresi harus tipe ordinal, misalnya maksimum \var{Longint}.
% Ini terjadi, misalnya ketika anda menetapkan argumen kedua
% ke \var{Inc} atau \var{Dec} yang tidak mengevaluasi ke nilai ordinal.
type_e_pointer_type_expected=04008_E_Tipe pointer diharapkan, tapi didapat "$1"
% Ekspresi variabel bukan tipe \var{pointer}. Ini terjadi ketika anda
% mengirimkan variabel yang bukan pointer ke \var{New} atau \var{Dispose}.
type_e_class_type_expected=04009_E_Tipe class diharapkan, tapi didapat "$1"
% Ekspresi variabel bukan tipe \var{class}. Ini terjadi biasanya ketika
% \begin{enumerate}
% \item Kelas leluhur dalam deklarasi kelas bukan sebuah class.
% \item Pengendali eksepsi (\var{On}) berisi pengenal tipe yang bukan sebuah 
% class.
% \end{enumerate}
type_e_cant_eval_constant_expr=04011_E_Tidak bisa mengevaluasi ekspresi konstan
% Kesalahan ini bisa terjadi ketika batas array yang anda deklarasikan tidak
% mengevaluasi konstan ordinal
type_e_set_element_are_not_comp=04012_E_Set elemen tidak kompatibel
% Anda mencoba untuk membuat operasi pada dua set, ketika set tipe elemen
% tidak sama. Tipe basis dari sebuah set harus sama saat mengambil union
type_e_set_operation_unknown=04013_E_Operasi tidak diimplementasikan untuk set
% Beberapa operasi biner tidak mendefinisikan untuk set seperti 
% div mod ** (juga >= <= untuk saat ini)
type_w_convert_real_2_comp=04014_W_Konversi tipe otomatis dari tipe floating ke COMP yang adalah tipe integer
% Konversi tipe implisit dari tipe real ke \var{comp} ditemukan. Karena
% \var{comp} adalah tipe integer 64 bit, ini bisa menandakan kesalahan.
type_h_use_div_for_int=04015_H_Sebaiknya gunakan DIV untuk mendapatkan hasil integer
% Ketika petunjuk dihidupkan, maka pembagian integer dengan operator '/' akan
% memunculkan pesan ini, karena hasil kemudian akan berupa tipe real
type_e_strict_var_string_violation=04016_E_Tipe string tidak sama, karena mode $V+
% Ketika mengompilasi dalam mode \var{\{\$V+\}}, string yang anda kirimkan sebagai parameter
% harus tipe yang sama persis seperti parameter yang dideklarasikan pada procedure.
type_e_succ_and_pred_enums_with_assign_not_possible=04017_E_succ atau pred pada enums dengan penempatan tidak mungkin
% Ketika anda mendeklarasikan tipe enumerasi yang mempunyai penempatan di dalamnya, seperti pada C,
% seperti yang berikut:
% \begin{verbatim}
%   Tenum = (a,b,e:=5);
% \end{verbatim}
% anda tidak bisas menggunakan fungsi \var{Succ} atau \var{Pred} padanya.
type_e_cant_read_write_type=04018_E_Tidak bisa membaca atau menulis variabel dari tipe ini
% Anda mencoba untuk \var{read} atau \var{write} variabel dari atau ke sebuah
% file tipe teks, yang tidak mendukung itu. Hanya tipe integer, real, pchar,
% dan string dapat dibaca dari/ditulis ke file teks. Boolean hanya dapat
% ditulis ke file teks.
type_e_no_readln_writeln_for_typed_file=04019_E_Tidak bisa menggunakan readln atau writeln pada file
% \var{readln} dan \var{writeln} hanya dibolehkan untuk file teks.
type_e_no_read_write_for_untyped_file=04020_E_Tidak bisa memakai baca atau tulis pada file untyped.
% \var{read} dan \var{write} hanya dibolehkan untuk file teks atau biner.
type_e_typeconflict_in_set=04021_E_Tipe konflik diantara set elemen
% Ada setidaknya satu set elemen yang tipenya salah, misalnya bukan dari
% tipe set.
type_w_maybe_wrong_hi_lo=04022_W_lo/hi(dword/qword) mengembalikan upper/lower word/dword
% \fpc mendukung versi di-overload atas \var{lo/hi} untuk \var{longint/dword/int64/qword}
% yang mengembalikan lower/upper word/dword pada argumen. TP selalu menggunakan
% 16 bit \var{lo/hi} yang selalu mengembalikan bit 0..7 untuk \var{lo} dan
% bit 8..15 untuk \var{hi}. Jika anda menginginkan perilaku TP anda harus
% type cast argumennya ke \var{word/integer}
type_e_integer_or_real_expr_expected=04023_E_Ekspresi integer atau real diharapkan
% Argumen pertama pada \var{str} harus tipe real atau integer.
type_e_wrong_type_in_array_constructor=04024_E_Tipe "$1" salah dalam konstruktor array
% Anda mencoba untuk menggunakan tipe dalam konstruktor array yang tidak
% dibolehkan.
type_e_wrong_parameter_type=04025_E_Tipe tidak kompatibel untuk arg no. $1: Didapat "$2", diharapkan "$3"
% Anda mencoba untuk mengirimkan tipe yang salah untuk parameter yang ditetapkan.
type_e_no_method_and_procedure_not_compatible=04026_E_Metode (variabel) dan Prosedur (variabel) tidak kompatibel
% Anda tidak bisa menempatkan sebuah metode ke variabel prosedur atau prosedur
% ke sebuah pointer metode.
type_e_wrong_math_argument=04027_E_Konstan yang dikirimkan ke fungsi math internal tidak benar
% Argumen konstan yang dikirimkan ke fungsi ln atau sqrt di luar jangkauan
% definisi pada fungsi ini.
type_e_no_addr_of_constant=04028_E_Tidak bisas mengambil alamat dari ekspresi konstan
% Tidak mungkin untuk mendapatkan alamat ekspresi konstan, karena tidak disimpan
% dalam memori. Anda dapat mencoba membuat tipe konstan. Kesalahan ini juga bisa
% ditampilkan jika anda mencoba mengirimkan properti ke parameter var.
type_e_argument_cant_be_assigned=04029_E_Argumen tidak bisa ditempatkan
% Hanya ekspresi yang bisa di sisi kiri dari penempatan yang bisa dikirimkan 
% sebagai panggilan dengan referensi argumen.
% Catatan: Properti hanya bisa dipakai pada sisi kiri dari penempatan,
% tapi ia tidak bisas digunakan sebagai argumen
type_e_cannot_local_proc_to_procvar=04030_E_Tidak bisa menempatkan prosedur/fungsi lokal ke variabel prosedur
% Tidak dibolehkan untuk menempatkan prosedur/fungsi lokal ke sebuah variabel
% prosedur, karena pemanggilan prosedur/fungsi lokal berbeda. Anda hanya bisa 
% menempatkan prosedur/fungsi lokal ke sebuah pointer.
type_e_no_assign_to_addr=04031_E_Tidak bisa menempatkan nilai ke alamat
% Tidak dibolehkan untuk menempatkan nilai ke alamat dari variabel, konstan,
% prosedur atau fungsi. Anda dapat mencoba mengompilasi dengan -So jika pengenal
% adalah variabel procedure.
type_e_no_assign_to_const=04032_E_Tidak bisa menempatkan nilai ke variabel const
% Tidak dibolehkan untuk menempatkan nilai ke variabel yang dideklarasikan
% sebagai const. Ini biasanya parameter dideklarasikan sebagai const, untuk
% membolehkan perubahan nilai buat parameter sebagai parameter nilai atau var.
type_e_array_required=04033_E_Tipe array diperlukan
% Jika anda mengakses variabel menggunakan sebuah indeks '[<x>]' maka tipenya
% harus array. Dalam mode FPC, pointer juga dibolehkan.
type_e_interface_type_expected=04034_E_tipe interface diharapkan, tapi didapat "$1"
% Kompilator berharap menemukan nama tipe interface, tapi didapatkan yang lain.
% Kode berikut akan menimbulkan kesalahan ini:
% \begin{verbatim}
% Type
%   TMyStream = Class(TStream,Integer)
% \end{verbatim}
type_h_mixed_signed_unsigned=04035_H_Mixing signed expressions and longwords gives a 64bit result
% If you divide (or calculate the modulus of) a signed expression by a longword (or vice versa),
% or if you have overflow and/or range checking turned on and use an arithmetic
% expression (+, -, *, div, mod) in which both signed numbers and longwords appear,
% then everything has to be evaluated in 64-bit arithmetic which is slower than normal
% 32-bit arithmetic. You can avoid this by typecasting one operand so it
% matches the result type of the other one.
type_w_mixed_signed_unsigned2=04036_W_Mencampur ekspresi bertanda dan cardinal di sini dapat menyebabkan kesalahan pemeriksaan jangkauan
% Jika anda menggunakan operator biner (and, or, xor) dan salah satu operand
% adalah longword sementara yang lainnya ekspresi bertanda, maka jika
% pemeriksaan jangkauan dihidupkan, anda akan mendapatkan kesalahan pemeriksaan
% jangkauan, karena kedua operand dikonversi ke longword sebelum operasi 
% dilakukan. Anda dapat menghindari ini dengan typecasting satu operand agar
% sesuai dengan tipe hasil atas yang lain.
type_e_typecast_wrong_size_for_assignment=04037_E_Typecast mempunyai ukuran berbeda ($1 -> $2) dalam penempatan
% Type casting ke sebuah tipe dengan ukuran berbeda tidak dibolehkan saat 
% variabel dipakai untuk penempatan.
type_e_array_index_enums_with_assign_not_possible=04038_E_enum dengan penempatan tidak bisa dipakai sebagai indeks array
% Ketika anda mendeklarasikan tipe enumerasi yang memiliki penempatan 
% didalamnya, seperti dalam C, sepeprti berikut:
% \begin{verbatim}
%   Tenum = (a,b,e:=5);
% \end{verbatim}
% anda tidak bisa menggunakannya sebagai indeks dari sebuah array.
type_e_classes_not_related=04039_E_Tipe Class atau Object "$1" dan "$2" tidak berkaitan
% Ada typecast dari satu kelas atau obyek ke yang lain sementara kelas/obyek itu
% tidak ada kaitannya. Ini akan menimbulkan kesalahan
type_w_classes_not_related=04040_W_Tipe kelas "$1" dan "$2" tidak berkaitan
% Ada typecast dari satu kelas atau obyek ke yang lain sementara kelas/obyek itu
% tidak ada kaitannya. Ini akan menimbulkan kesalahan
type_e_class_or_interface_type_expected=04041_E_Tipe Class atau interface diharapkan, tapi didapat "$1"
% Kompilator mengharapkan nama kelas atau interface, tapi mendapatkan tipe atau pengenal lain.
type_e_type_is_not_completly_defined=04042_E_Tipe "$1" tidak sepenuhnya didefinisikan
% Kesalahan ini terjadi saat tipe tidak lengkap: misalnya tipe pointer yang
% mengarah ke tipe tidak terdefinisi.
type_w_string_too_long=04043_W_Literal string memiliki lebih banyak karakter daripada panjang short string
% Ukuran string konstan, yang ditempatkan ke shortstring, lebih panjang daripada
% ukuran maksimum shortstring
type_w_comparison_always_false=04044_W_Comparison might be always false due to range of constant and expression
% There is a comparison between a constant and an expression where the constant is out of the
% valid range of values of the expression. Because of type promotion, the statement will always evaluate to
% false. Explicitly typecast the constant or the expression to the correct range to avoid this warning
% if you think the code is correct.
type_w_comparison_always_true=04045_W_Comparison might be always true due to range of constant and expression
% There is a comparison between a constant and an expression where the constant is out of the
% valid range of values of the expression. Because of type promotion, the statement will always evaluate to
% true. Explicitly typecast the constant or the expression to the correct range to avoid this warning
% if you think the code is correct.
type_w_instance_with_abstract=04046_W_Membentuk sebuah kelas "$1" dengan metode abstrak
% Turunan kelas dibuat yang berisi metode abstrak tidak diimplementasikan. Ini
% dapat membawa kesalahan runtime 211 dalam kode jika rutin itu tidak pernah
% dibersihkan. Semua metode abstrak harus diganti.
type_h_in_range_check=04047_H_Operand kiri pada operator IN harus berukuran byte
% Operand kiri pada operator \var{in} bukan ordinal atau enumerasi yang sesuai
% dalam 8-bits, ini dapat mengakibatkan kesalahan pemeriksaan jangkauan.
% Operator \var{in} saat ini hanya mendukung operand kiri yang sesuai dalam byte.
% Dalam hal enumerasi, ukuran elemen pada enumerasi dapat dikontrol dengan
% saklar \var{\{\$PACKENUM\}} atau \var{\{\$Zn\}}.
type_w_smaller_possible_range_check=04048_W_Ukuran tipe tidak sama, kemungkinan kehilangan data / kesalahan pemeriksaan jangkauan
% Ada penempatan ke tipe lebih kecil daripada tipe sumber. Ini berarti bahwa
% ini dapat menyebabkan kesalahan pemeriksaan jangkauan, atau dapat mengakibatkan kehilangan data.
type_h_smaller_possible_range_check=04049_H_Ukuran tipe tidak sama, kemungkinan kehilangan data / kesalahan pemeriksaan jangkauan
% Ada penempatan ke tipe lebih kecil daripada tipe sumber. Ini berarti bahwa
% ini dapat menyebabkan kesalahan pemeriksaan jangkauan, atau dapat mengakibatkan kehilangan data.
type_e_cant_take_address_of_abstract_method=04050_E_Alamat metode abstrak tidak bisa diambil
% Metode abstrak tidak memiliki badan, maka alamat metode abstrak tidak bisa diambil.
type_e_assignment_not_allowed=04051_E_Penempatan ke parameter formal dan array terbuka tidak mungkin
% Anda mencoba untuk menempatkan nilai ke parameter formal (untyped var, const
% atau out), atau ke array terbuka.
type_e_constant_expr_expected=04052_E_Ekspresi Konstan diharapkan
% Kompilator mengharapkan ekspresi konstan, tapi mendapatkan ekspresi variabel.
type_e_operator_not_supported_for_types=04053_E_Operasi "$1" tidak didukung untuk tipe "$2" dan "$3"
% Operasi tidak dibolehkan untuk tipe yang disediakan
type_e_illegal_type_conversion=04054_E_Konversi tipe tidak benar: "$1" ke "$2"
% Ketika melakukan type-cast, anda harus berhati-hati bahwa ukuran variabel dan tipe
% tujuan adalah sama.
type_h_pointer_to_longint_conv_not_portable=04055_H_Konversi antara ordinal dan pointer tidak portabel
% Jika anda melakukan typecast pointer ke longint (atau sebaliknya), kode ini
% tidak akan dikompilasi pada mesin menggunakan 64-bit untuk penyimpanan pointer.
type_w_pointer_to_longint_conv_not_portable=04056_W_Konversi antara ordinal dan pointer tidak portabel
% Jika anda melakukan typecast pointer ke tipe ordinal pada ukuran berbeda
% (atau sebaliknya), ini dapat menyebabkan masalah. Ini adalah peringatan untuk
% membantu menemukan kode spesifik 32bit di mana cardinal/longint dipakai
% untuk typecast pointer ke ordinal. Solusinya sebaiknya menggunakan tipe ptrint/ptruint.
type_e_cant_choose_overload_function=04057_E_Tidak dapat menentukan fungsi di-overload mana yang dipanggil
% Anda memanggil fungsi di-overload dengan parameter yang tidak terkait ke daftar
% parameter fungsi yang dideklarasikan di manapun. Misalnya saat anda
% mendeklarasikan fungsi dengan parameter \var{word} dan \var{longint}, kemudian
% anda memanggilnya dengan parameter yang tipenya \var{integer}.
type_e_illegal_count_var=04058_E_Variabel penghitung tidak benar
% Tipe variabel loop \var{for} harus tipe ordinal. Variabel loop tidak bisa
% real atau string.
type_w_double_c_varargs=04059_W_Mengkonversi konstan nilai real ke double untuk argumen variabel C, tambah typecast eksplisit untuk menghindarinya.
% Dalam C, nilai konstan real adalah double standarnya. Untuk alasan ini, jika
% anda mengirimkan nilai konstan real ke argumen variabel bagian dari fungsi C,
% standarnya FPC mengkonversi konstan ini ke double juga. Jika anda ingin
% menghindari ini terjadi, tambah typecast eksplisit disekitar konstan.
type_e_class_or_cominterface_type_expected=04060_E_Tipe Class atau interface COM diharapkan, tapi didapat "$1"
% Beberapa operator seperti operator AS hanya berlaku pada kelas atau interface COM.
type_e_no_const_packed_array=04061_E_Konstan packed arrays belum didukung
% Anda tidak bisa mendeklarasikan (bit)packed array sebagai konstan type.
type_e_got_expected_packed_array=04062_E_Tipe tidak kompatibel untuk arg no. $1: Didapat "$2" diharapkan "(Bit)Packed Array"
% Kompilator mengharapkan (bit)packed array sebagai parameter yang dietapkan
type_e_got_expected_unpacked_array=04063_E_Tipe tidak kompatibel untuk arg no. $1: Didapat "$2" diharapkan "(Bit)Packed Array"
% Kompilator mengharapkan reguler (misalnya bukan packed) array sebagai parameter yang ditetapkan
type_e_no_packed_inittable=04064_E_Elemen pada packed arrays tidak bisa dari tipe yang perlu diinisialisasi
% Dukungan untuk packed array daru tuoe yang memerlukan inisialisasi (seperti
% ansistring, atau record yang berisi ansistring) belum diimplementasikan.
type_e_no_const_packed_record=04065_E_Konstan packed record dan obyek belum didukung
% Anda tidak bisa mendeklarasikan (bit)packed array sebagai typed konstan pada saat ini.
type_w_untyped_arithmetic_unportable=04066_W_Aritmatika "$1" pada pointer untyped tidak portabel bagi {$T+}, disarankan typecast
% Penambahan/pengurangan dari pointer untyped mungkin bekerja secara berbeda
% dalam \var{\{\$T+\}}, gunakan typecast untuk pointer typed
type_e_cant_take_address_of_local_subroutine=04076_E_Tidak bisa mengambil alamat dari subrutin yang ditandai sebagai lokal
% Alamat subrutin yang ditandai sebagai lokal tidak bisa diambil.
type_e_cant_export_local=04077_E_Tidak bisa mengekspor subrutin yang ditandai sebagai lokal dari sebuah unit
% Subrutin yang ditandai sebagai lokal tidak bisa diekspor dari sebuah unit.
type_e_not_automatable=04078_E_Tipe tidak bisa diotomasi: "$1"
% Hanya byte, integer, longint, smallint, currency, single, double, ansistring,
% widestring, tdatetime, variant, olevariant, wordbool dan semua interface bisa diotomasi.
type_h_convert_add_operands_to_prevent_overflow=04079_H_Mengkonversi operand ke "$1" sebelum melakukan penambahan dapat menghindari kesalahan overflow.
% Menambah dua tipe dapat menyebabkan kesalahan overflow. Karena anda
% mengkonversi hasil ke tipe lebih besar, anda dapat menghindari kesalahan
% seperti itu dengan mengubah operand ke tipe ini sebelum melakukan penambahan.
type_h_convert_sub_operands_to_prevent_overflow=04080_H_Mengkonversi operand ke "$1" sebelum melakukan pengurangan dapat menghindari kesalahan overflow.
% Pengurangan dua tipe dapat menyebabkan kesalahan overflow. Karena anda
% mengkonversi hasil ke tipe lebih besar, anda menghindari 
% seperti itu dengan mengubah operand ke tipe ini sebelum melakukan pengurangan.
type_h_convert_mul_operands_to_prevent_overflow=04081_H_Mengkonversi operand ke "$1" sebelum melakukan perkalian dapat menghindari kesalahan overflow.
% Perkalian dua tipe dapat menyebabkan kesalahan overflow. Karena anda
% mengkonversi hasil ke tipe lebih besar, anda menghindari 
% seperti itu dengan mengubah operand ke tipe ini sebelum melakukan pengurangan.
type_w_pointer_to_signed=04082_W_Mengkonversi pointer ke integer bertanda dapat membuat hasil perbandingan yang salah dan kesalahan jangkauan, sebaiknya gunakan tipe unsigned.
% Ruang alamat virtual pada mesin 32-bit berjalan dari \$00000000 sampai
% \$ffffffff. Banyak sistem operasi membeolehkan anda untuk mengalokasikan
% memori di atas \$80000000, sebagai contoh Windows dan Linux membolehkan
% pointer dalam jangkauan \$0000000 sampai \$bfffffff. Jika anda mengkonversi
% pointer ke tipe bertanda, ini dapat menyebabkan overflow dan kesalahan
% pemeriksaan jangkauan, tapi juga \$80000000 < \$7fffffff.
% Ini bisa menyebabkan kesalahan acak dalam kode seperti "if p>q".
% \end{description}
#
# Symtable
#
# 05060 is the last used one
#
% \section{Penanganan simbol}
% Seksi ini mendaftarkan semua pesan yang berkenaan dengan penanganan simbol.
% Ini berarti semua hal yang harus dilakukan dengan prosedur dan nama variabel.
% \begin{description}
type_e_interface_has_no_guid=04083_E_Interface type $1 has no valid GUID
% When applying the as-operator to an interface or class, the desired interface (i.e. the right operand of the
% as-operator) must have a valid GUID.
type_e_invalid_objc_selector_name=04084_E_Invalid selector name "$1"
% An Objective-C selector cannot be empty, must be a valid identifier or a single colon,
% and if it contains at least one colon it must also end in one.
type_e_expected_objc_method_but_got=04085_E_Expected Objective-C method, but got $1
% A selector can only be created for Objective-C methods, not for any other kind
% of procedure/function/method.
type_e_expected_objc_method=04086_E_Expected Objective-C method or constant method name
% A selector can only be created for Objective-C methods, either by specifying
% the name using a string constant, or by using an Objective-C method identifier
% that is visible in the current scope.
type_e_no_type_info=04087_E_No type info available for this type
% Type information is not generated for some types, such as enumerations with gaps
% in their value range (this includes enumerations whose lower bound is different
% from zero).
type_e_ordinal_or_string_expr_expected=04088_E_Ordinal or string expression expected
% The expression must be an ordinal or string type.
type_e_string_expr_expected=04089_E_String expression expected
% The expression must be a string type.
type_w_zero_to_nil=04090_W_Converting 0 to NIL
% Use NIL rather than 0 when initialising a pointer.
type_e_protocol_type_expected=04091_E_Objective-C protocol type expected, but got "$1"
% The compiler expected a protocol type name, but found something else.
type_e_objc_type_unsupported=04092_E_The type "$1" is not supported for interaction with the Objective-C and the blocks runtime.
% Objective-C and Blocks make extensive use of run time type information (RTTI). This format
% is defined by the maintainers of the run time and can therefore not be adapted
% to all possible Object Pascal types. In particular, types that depend on
% reference counting by the compiler (such as ansistrings and certain kinds of
% interfaces) cannot be used as fields of Objective-C classes, cannot be
% directly passed to Objective-C methods or Blocks, and cannot be encoded using \var{objc\_encode}.
type_e_class_or_objcclass_type_expected=04093_E_Class or objcclass type expected, but got "$1"
% It is only possible to create class reference types of \var{class} and \var{objcclass}
type_e_objcclass_type_expected=04094_E_Objcclass type expected
% The compiler expected an \var{objcclass} type
type_w_procvar_univ_conflicting_para=04095_W_Coerced univ parameter type in procedural variable may cause crash or memory corruption: $1 to $2
% \var{univ} parameters are implicitly compatible with all types of the same size,
% also in procedural variable definitions. That means that the following code is
% legal, because \var{single} and \var{longint} have the same size:
% \begin{verbatim}
% {$mode macpas}
% Type
%   TIntProc = procedure (l: univ longint);
%
%   procedure test(s: single);
%     begin
%       writeln(s);
%     end;
%
%   var
%     p: TIntProc;
%   begin
%     p:=test;
%     p(4);
%   end.
% \end{verbatim}
% This code may however crash on platforms that pass integers in registers and
% floating point values on the stack, because then the stack will be unbalanced.
% Note that this warning will not flag all potentially dangerous situations.
% when \var{test} returns.
type_e_generics_cannot_reference_itself=04096_E_Type parameters of specializations of generics cannot reference the currently specialized type
% Recursive specializations of generics like \var{Type MyType = specialize MyGeneric<MyType>;} are not possible.
type_e_type_parameters_are_not_allowed_here=04097_E_Type parameters are not allowed on non-generic class/record/object procedure or function
% Type parameters are only allowed for methods of generic classes, records or objects
type_e_generic_declaration_does_not_match=04098_E_Generic declaration of "$1" differs from previous declaration
% Generic declaration does not match the previous declaration
type_e_helper_type_expected=04099_E_Helper type expected
% The compiler expected a \var{class helper} type.
type_e_record_type_expected=04100_E_Record type expected
% The compiler expected a \var{record} type.
type_e_class_helper_must_extend_subclass=04101_E_Derived class helper must extend a subclass of "$1" or the class itself
% If a class helper inherits from another class helper the extended class must
% extend either the same class as the parent class helper or a subclass of it
type_e_record_helper_must_extend_same_record=04102_E_Derived record or type helper must extend "$1"
% If a record helper inherits from another record helper it must extend the same
% record that the parent record helper extended.
type_e_procedures_return_no_value=04103_E_Invalid assignment, procedures return no value
% This error occurs when one tries to assign the result of a procedure or destructor call.
% A procedure or destructor returns no value so this is not
% possible.
type_w_implicit_string_cast=04104_W_Implicit string type conversion from "$1" to "$2"
% An implicit type conversion from an ansi string type to an unicode string type is
% encountered. To avoid this warning perform an explicit type conversion.
type_w_implicit_string_cast_loss=04105_W_Implicit string type conversion with potential data loss from "$1" to "$2"
% An implicit type conversion from an unicode string type to an ansi string type is
% encountered. This conversion can lose data since not all unicode characters may be represented in the codepage of
% destination string type.
type_w_explicit_string_cast=04106_-W_Explicit string typecast from "$1" to "$2"
% An explicit typecast from an ansi string type to an unicode string type is
% encountered. This warning is off by default. You can turn it on to see all suspicious string conversions.
type_w_explicit_string_cast_loss=04107_-W_Explicit string typecast with potential data loss from "$1" to "$2"
% An explicit typecast from an unicode string type to an ansi string type is
% encountered. This conversion can lose data since not all unicode characters may be represented in the codepage of
% destination string type. This warning is off by default. You can turn it on to see all the places with lossy string
% conversions.
type_w_unicode_data_loss=04108_W_Unicode constant cast with potential data loss
% Conversion from a WideChar to AnsiChar can lose data since now all unicode characters may be represented in the current
% system codepage
% You can nest function definitions only 31 levels deep.
type_e_range_check_error_bounds=04109_E_range check error while evaluating constants ($1 must be between $2 and $3)
type_w_range_check_error_bounds=04110_W_range check error while evaluating constants ($1 must be between $2 and $3)
% The constants are outside their allowed range.
type_e_type_not_allowed_for_default=04111_E_This type is not supported for the Default() intrinsic
% Some types like for example Text and File Of X are not supported by the Default intrinsic.
type_e_java_class_method_not_static_virtual=04112_E_JVM virtual class methods cannot be static
% Virtual class methods cannot be static when targeting the JVM platform, because
% the self pointer is required for correct dispatching.
type_e_invalid_final_assignment=04113_E_Final (class) fields can only be assigned in their class' (class) constructor
% It is only possible to assign a value to a final (class) field inside a (class) constructor of its owning class.
type_e_no_managed_formal_assign_typecast=04114_E_It is not possible to typecast untyped parameters on managed platforms, simply assign a value to them instead.
% On managed platforms, untyped parameters are translated by the compiler into
% the equivalent of \var{var x: BaseClassType}. Non-class-based types passed to
% such parameters are automatically wrapped (or boxed) in a class, and after the
% call the potentially modified value is assigned back to the original variable.
% On the caller side, changing untyped var/out parameters happens by simply assigning
% values to them (either class-based or primitive ones). On the caller side,
% they will be extracted and if their type does not match the original variable's,
% an exception will be raised.
type_e_no_managed_assign_generic_typecast=04115_E_The assignment side of an expression cannot be typecasted to a supertype on managed platforms
% Managed platforms guarantee type safety at the bytecode level. This means that the virtual machine must be able
% to statically determine that no type-unsafe assignments or operations occur. By assigning a parent class type to a
% variable of a child type by typecasting the assignment side to the parent class type, the type safety would no
% longer be guaranteed and the generated code would fail verification at run time time.
type_w_interface_lower_visibility=04116_-W_The interface method "$1" raises the visibility of "$2" to public when accessed via an interface instance
type_e_interface_lower_visibility=04117_E_The interface method "$1" has a higher visibility (public) than "$2"
% All methods in an interface have always public visibility. That means that if
% an interface method is implemented using a (strict) protected or private method,
% this method is actually publicly accessible via the interface. On the JVM
% target this situation results in an error because the JVM rejects such
% attempts to circumvent the visibility rules. On other targets this is a
% warning that is disabled by default because such situations are common
% practice, but it can be enabled in case you are concerned with keeping your
% code compilable for the JVM target.
type_e_typeof_requires_vmt=04118_E_TYPEOF can only be used on object types with VMT
% Typeof() intrinsic returns pointer to VMT of its argument. It cannot be used on object types that do not have VMT.
type_e_invalid_default_value=04119_E_It is not possible to define a default value for a parameter of type "$1"
% Parameters declared as structured types, such as files, variants, non-dynamic
% arrays and TP-style objects, cannot have a default value.
type_e_type_not_allowed_for_type_helper=04120_E_Type "$1" cannot be extended by a type helper
% Types like procedural variables cannot be extended by type helpers
type_e_procedure_must_be_far=04121_E_Procedure or function must be far in order to allow taking its address: "$1"
% In certain i8086 memory models (medium, large and huge), procedures and functions
% have to be declared 'far' in order to allow their address to be taken.
type_w_instance_abstract_class=04122_W_Creating an instance of abstract class "$1"
% The specified class is declared as \var{abstract} and thus no instance of this class
% should be created. This is merely a warning for Delphi compatibility.
type_e_function_reference_kind=04123_E_Subroutine references cannot be declared as "of object" or "is nested", they can always refer to any kind of subroutine
% Subroutine references can refer to any kind of subroutine and hence do not
% require specialisation for methods or nested subroutines.
type_e_anonymous_function_unsupported=04999_E_Function references are not yet supported, only blocks (add "cdecl;" at the end)
% Remove this error message once Delphi-style anonymous are implemented. It has
% number 4999 so as not to result in a gap in the error message numbering once
% it's removed.
% \end{description}
#
# Symtable
#
# 05095 is the last used one
#
% \section{Symbol handling}
% This section lists all the messages that concern the handling of symbols.
% This means all things that have to do with procedure and variable names.
% \begin{description}
sym_e_id_not_found=05000_E_Pengenal tidak ditemukan "$1"
% Kompilator tidak mengetahui simbol ini. Biasanya terjadi ketika anda salah
% mengeja nama variabel atau prosedur, atau ketika anda lupa untuk
% mendeklarasikan variabel.
sym_f_internal_error_in_symtablestack=05001_F_Kesalahan Internal dalam SymTableStack()
% Kesalahan internal terjadi dalam kompilator; Jika anda menemukan kesalahan ini,
% silahkan hubungi para pengembang dan coba untuk menyertakan penjelasan pasti
% dari keadaan di mana kesalahan terjadi.
sym_e_duplicate_id=05002_E_Duplikasi pengenal "$1"
% Pengenal sudah dideklarasikan dalam lingkup saat ini.
sym_h_duplicate_id_where=05003_H_Pengenal sudah didefinisikan dalam $1 pada baris $2
% Pengenal sudah dideklarasikan dalam lingkup sebelumnya.
sym_e_unknown_id=05004_E_Pengenal "$1" tidak dikenal
% Pengenal ditemukan belum dideklarasikan, atai dipakai di luar lingkup
% di mana ia didefinisikan.
sym_e_forward_not_resolved=05005_E_Deklarasi fotidak memecahkan "$1"
% Ini dapat terjadi dalam dua kasus:
% \begin{itemize}
% \item Ini terjadi saat anda mendeklarasikan fungsi bagian (dalam \var{interface},
% atau dengan direktif \var{forward}, tapi tidak mengimplementasikannya.
% \item Anda mereferensi tipe yang tidak dideklarasikan dalam blok \var{type}
% saat ini.
% \end{itemize}
sym_e_error_in_type_def=05007_E_Kesalahan dalam definisi tipe
% Ada kesalahan dalam definisi anda pada tipe array baru:
% \item Salah satu pembatas jangkauan dalam deklarasi array salah.
% Sebagai contoh, \var{Array [1..1.25]} akan memicu kesalahan ini.
sym_e_forward_type_not_resolved=05009_E_Tipe forward tidak memecahkan "$1"
% Simbol forward didefinisikan, tapi tidak ada deklarasi yang ditemukan.
sym_e_only_static_in_static=05010_E_Hanya variabel statis bisa dipakai dalam metode statis atau di luar metode
% Metode statis pada obyek hanya bisa mengakses variabel statis.
sym_e_type_must_be_rec_or_object_or_class=05012_E_Record or object or class type expected
% The variable or expression isn't of the type \var{record} or \var{object} or \var{class}.
sym_e_no_instance_of_abstract_object=05013_E_Turunan kelas atau obyek dengan metode abstrak tidak dibolehkan
% Anda mencoba untuk membuat turunan kelas yang memiliki metode abstrak yang
% tidak diganti.
sym_w_label_not_defined=05014_W_Label tidak didefinisikan "$1"
% Label dideklarasikan, tapi tidak didefinisikan.
sym_e_label_used_and_not_defined=05015_E_Label dipakai tapi tidak didefinisikan "$1"
% Label dideklarasikan dan dipakai, tapi tidak didefinisikan.
sym_e_ill_label_decl=05016_E_Deklarasi label tidak benar
% Kesalahan ini seharusnya tidak pernah terjadi; ia terjadi jika label 
% didefinisikan di luar prosedur atau fungsi.
sym_e_goto_and_label_not_supported=05017_E_GOTO dan LABEL tidak didukung (gunakan saklar -Sg)
% Anda harus mengompilasi program yang mempunyai pernyataan \var{label}s dan \var{goto}
% dengan saklar \var{-Sg}. Standarnya, \var{label} dan \var{goto} tidak
% didukung.
sym_e_label_not_found=05018_E_Label tidak ditemukan
% \var{goto label} ditemukan, tapi label tidak dideklarasikan.
sym_e_id_is_no_label_id=05019_E_pengenal bukan sebuah label
% Pengenal ditetapkan setelah \var{goto} bukan tipe label.
sym_e_label_already_defined=05020_E_label sudah didefinisikan
% Anda mendefinisikan label dua kali. Anda bisa mendefinisikan label hanya sekali.
sym_e_ill_type_decl_set=05021_E_deklarasi tipe tidak benar pada set elemen
% Deklarasi sebuah set berisi definisi tipe yang tidak benar.
sym_e_class_forward_not_resolved=05022_E_Definisi kelas forward tidak memecahkan "$1"
% Anda mendeklarasikan kelas, tapi anda tidak mengimplementasikannya.
sym_n_unit_not_used=05023_H_Unit "$1" tidak dipakai dalam $2
% Unit yang direferensi ulang dalam klausul \var{uses} tidak dipakai.
sym_h_para_identifier_not_used=05024_H_Parameter "$1" tidak dipakai
% Pengenal dideklarasikan (secara lokal atau global) tapi tidak dipakai
% (secara lokal atau global).
sym_n_local_identifier_not_used=05025_N_Variabel lokal "$1" tidak dipakai
% Anda telah mendeklarasikan, tapi tidak menggunakan variabel dalam implementasi
% prosedur atau fungsi.
sym_h_para_identifier_only_set=05026_H_Parameter nilai "$1" ditempati tetapi tidak dipakai
% Pengenal dideklarasikan (secara lokal atau global) dan disetel tapi tidak
% dipakai (secara lokal atau global).
sym_n_local_identifier_only_set=05027_N_Variabel lokal "$1" ditempati tapi tidak dipakai
% Implementasi variabel dalam prosedur atau fungsi dideklarasikan, disetel
% tapi tidak pernah digunakan.
sym_h_local_symbol_not_used=05028_H_Lokal $1 "$2" tidak dipakai
% Simbol lokal tidak pernah digunakan.
sym_n_private_identifier_not_used=05029_N_Field private "$1.$2" tidak pernah dipakai
% Field private didefinisikan, tapi tidak pernah dipakai dalam kode.
sym_n_private_identifier_only_set=05030_N_Field private "$1.$2" ditempati tapi tidak pernah dipakai
% Field private dideklarasikan, ditempati tapi tidak pernah dibaca.
sym_n_private_method_not_used=05031_N_Metode private "$1.$2" tidak pernah dipakai
% Metode private dideklarasikan tapi tidak pernah dipakai dalam kode.
sym_e_set_expected=05032_E_Tipe set diharapkan
% Variabel atau ekspresi bukan tipe \var{set}. Ini terjadi dalam pernyataan
% \var{in}.
sym_w_function_result_not_set=05033_W_Hasil fungsi nampaknya belum disetel
% Anda mendapatkan peringatan ini jika kompilator berpikir bahwa nilai kembalian
% fungsi tidak disetel. Ini tidak akan ditampilkan untuk prosedur assembler,
% atau prosedur yang berisi blok assembler.
sym_w_wrong_C_pack=05034_W_Tipe "$1" tidak dijajarkan dengan benar dalam record saat ini untuk C
% Array dengan ukuran tidak multipel dari 4 akan dijajarkan secara salah untuk
% struktur C.
sym_e_illegal_field=05035_E_Pengenal field record "$1" tidak dikenal
% Field tidak ada dalam definisi record/object.
sym_w_uninitialized_local_variable=05036_W_Variabel lokal "$1" nampaknya belum diinisialisasi
% Pesan ini ditampilkan jika kompilator berpikir bahwa variabel akan dipakai
% (misalnya muncul dalam ekspresi sisi-tangan-kanan) saat ia tidak 
% diinisialisasi lebih dulu (misalnya muncul dalam sisi tangan-kiri pada
% penempatan)
sym_w_uninitialized_variable=05037_W_Variabel "$1" nampaknya belum diinisialisasi
% Pesan ini ditampilkan jika kompilator berpikir bahwa variabel akan dipakai
% (misalnya muncul dalam sisi tangan-kanan dari ekspresi) saat ia tidak
% diinisialisasi lebih dulu (misalnya muncul dalam sisi tangan-kiri pada
% penempatan)
sym_e_id_no_member=05038_E_pengenal idents tanpa anggota "$1"
% Kesalahan ini dibuat ketika pengenal record, field, atau metode diakses
% sementara ia tidak didefinisikan.
sym_h_param_list=05039_H_Ditemukan deklarasi: $1
% Anda mendapatkan ini ketika anda menggunakan saklar \var{-vh}. Dalam hal
% prosedur yang di-overload tidak ditemukan, maka semua kandidat prosedur yang
% di-overload didaftarkan, dengan daftar parameternya.
sym_e_segment_too_large=05040_E_Elemen data terlalu besar
% Anda mendapatkan in ketika anda mendeklarasikan elemen data yang ukurannya
% melebihi batas yang ditetapkan (2 Gb pada prosesor 80386+/68020+)
sym_e_no_matching_implementation_found=05042_E_Tidak ada implementasi yang sesuai untuk metode interface "$1" yang ditemukan
% Tidak ada metode yang sama ditemukan yang dapat mengimplementasi metode 
% interface. Periksa tipe argumen dan tipe hasil pada metode.
sym_w_deprecated_symbol=05043_W_Simbol "$1" tidak dipakai lagi
% Ini berarti bahwa simbol (variabel, rutin, dll...) yang dideklarasikan
% sebagai \var{deprecated} diterbitkan. Simbol deprecated mungkin tidak lagi
% tersedia dalam versi lebih baru atas unit / librari. Penggunaan simbol
% harus dihindari sebaik mungkin.
sym_w_non_portable_symbol=05044_W_Simbol "$1" tidak portabel
% Ini berarti bahwa sebuah simbol (variabel, rutin, dll...) yang dideklarasikan
% sebagai \var{platform} dipakai. Nilai simbol ini, penggunaannya serta
% ketersediaannya adalah spesifik platform dan tidak boleh dipakai jika kode
% sumber harus portabel.
sym_w_non_implemented_symbol=05055_W_Simbol "$1" tidak diimplementasikan
% Ini berarti bahwa sebuah simbol (variabel, rutin, dll...) yang dideklarasikan
% sebagai \var{unimplemented} dipakai. Simbol ini didefinisikan, tapi belum
% diimplementasikan pada spesifik platform ini.
sym_e_cant_create_unique_type=05056_E_Tidak bisa membuat tipe unik dari tipe ini
% Hanya tipe sederhana seperti ordinal, float dan tipe string didukung ketika
% mendefinisikan ulang tipe dengan \var{type newtype = type oldtype;}.
sym_h_uninitialized_local_variable=05057_H_Variabel lokal "$1" nampaknya belum diinisialisasi
% Pesan ini ditampilkan jika kompilator berpikir bahwa variabel yang akan
% dipakai (misalnya muncul dalam sisi-tangan-kiri atas ekspresi) saat ia tidak
% diinisialisasi lebih dulu (misalnya muncul dalam sisi tangan-kiri pada
% penempatan)
sym_h_uninitialized_variable=05058_H_Variabel "$1" nampaknya tidak diinisialisasi
% Pesan ini ditampilkan jika kompilator berpikir bahwa variabel yang akan
% dipakai (misalnya muncul dalam sisi-tangan-kiri atas ekspresi) saat ia tidak
% diinisialisasi lebih dulu (misalnya muncul dalam sisi tangan-kiri pada
% penempatan)
sym_w_function_result_uninitialized=05059_W_Variabel hasil fungsi nampaknya tidak diinisialisasi
% Pesan ini ditampilkan jika kompilator berpikir bahwa variabel hasil fungsi yang
% akan dipakai (misalnya muncul dalam sisi-tangan-kiri atas ekspresi) saat ia
% tidak diinisialisasi lebih dulu (misalnya muncul dalam sisi tangan-kiri pada
% penempatan)
sym_h_function_result_uninitialized=05060_H_Variabel hasil fungsi nampaknya tidak diinisialisasi
% Pesan ini ditampilkan jika kompilator berpikir bahwa variabel hasil fungsi
% yang akan dipakai (misalnya muncul dalam sisi-tangan-kanan atas ekspresi) saat
% tidak diinisialisasi lebih dulu (misalnya muncul dalam sisi tangan-kanan pada
% penempatan)
sym_w_identifier_only_read=05061_W_Variabel "$1" dibaca tapi tida ditempati dimanapun
% Anda membaca nilai sebuah variabel, tapi tidak menempatkan nilainya ke
% manapun.
sym_h_abstract_method_list=05062_H_Ditemukan metode abstrak: $1
% Ketika mendapatkan peringatan mengenai pembentukan class/object dengan metode
% abstrak anda mendapatkan petunjuk ini untuk menemukan metode yang dipengaruhi.
% \end{description}
#
# Codegenerator
#
# 06040 is the last used one
#
% \section{Pesan pembuat kode}
% Seksi ini mendaftarkan semua pesan yang dapat ditampilkan jika generator
% kode menemukan kondisi kesalahan.
% \begin{description}
sym_w_experimental_symbol=05063_W_Symbol "$1" is experimental
% This means that a symbol (a variable, routine, etc...) which is
% declared as \var{experimental} is used. Experimental symbols
% might disappear or change semantics in future versions. Usage of this symbol
% should be avoided as much as possible.
sym_w_forward_not_resolved=05064_W_Forward declaration "$1" not resolved, assumed external
% This happens if you declare a function in the \var{interface} of a unit in macpas mode,
% but do not implement it.
sym_w_library_symbol=05065_W_Symbol "$1" is belongs to a library
% This means that a symbol (a variable, routine, etc...) which is
% declared as \var{library} is used. Library symbols may not be
% available in other libraries.
sym_w_deprecated_symbol_with_msg=05066_W_Symbol "$1" is deprecated: "$2"
% This means that a symbol (a variable, routine, etc...) which is
% declared as \var{deprecated} is used. Deprecated symbols may no longer
% be available in newer versions of the unit / library. Use of this symbol
% should be avoided as much as possible.
sym_e_no_enumerator=05067_E_Cannot find an enumerator for the type "$1"
% This means that compiler cannot find an appropriate enumerator to use in the for-in loop.
% To create an enumerator you need to define an operator enumerator or add a public or published
% GetEnumerator method to the class or object definition.
sym_e_no_enumerator_move=05068_E_Cannot find a "MoveNext" method in enumerator "$1"
% This means that compiler cannot find a public MoveNext method with the Boolean return type in
% the enumerator class or object definition.
sym_e_no_enumerator_current=05069_E_Cannot find a "Current" property in enumerator "$1"
% This means that compiler cannot find a public Current property in  the enumerator class or object
% definition.
sym_e_objc_para_mismatch=05070_E_Mismatch between number of declared parameters and number of colons in message string.
% In Objective-C, a message name automatically contains as many colons as parameters.
% In order to prevent mistakes when specifying the message name in FPC, the compiler
% checks whether this is also the case here. Note that in case of messages taking a
% variable number of arguments translated to FPC via an \var{array of const} parameter,
% this final \var{array of const} parameter is not counted. Neither are the hidden
% \var{self} and \var{\_cmd} parameters.
sym_n_private_type_not_used=05071_N_Private type "$1.$2" never used
% The indicated private type is declared but is never used in the code.
sym_n_private_const_not_used=05072_N_Private const "$1.$2" never used
% The indicated private const is declared but is never used in the code.
sym_n_private_property_not_used=05073_N_Private property "$1.$2" never used
% The indicated private property is declared but is never used in the code.
sym_w_deprecated_unit=05074_W_Unit "$1" is deprecated
% This means that a unit which is
% declared as \var{deprecated} is used. Deprecated units may no longer
% be available in newer versions of the library. Use of this unit
% should be avoided as much as possible.
sym_w_deprecated_unit_with_msg=05075_W_Unit "$1" is deprecated: "$2"
% This means that a unit which is
% declared as \var{deprecated} is used. Deprecated units may no longer
% be available in newer versions of the library. Use of this unit
% should be avoided as much as possible.
sym_w_non_portable_unit=05076_W_Unit "$1" is not portable
% This means that a unit which is
% declared as \var{platform} is used. This unit use
% and availability is platform specific and should not be used
% if the source code must be portable.
sym_w_library_unit=05077_W_Unit "$1" is belongs to a library
% This means that a unit which is
% declared as \var{library} is used. Library units may not be
% available in other libraries.
sym_w_non_implemented_unit=05078_W_Unit "$1" is not implemented
% This means that a unit which is
% declared as \var{unimplemented} is used. This unit is defined,
% but is not yet implemented on this specific platform.
sym_w_experimental_unit=05079_W_Unit "$1" is experimental
% This means that a unit which is
% declared as \var{experimental} is used. Experimental units
% might disappear or change semantics in future versions. Usage of this unit
% should be avoided as much as possible.
sym_e_formal_class_not_resolved=05080_E_No complete definition of the formally declared class "$1" is in scope
% Objecive-C and Java classes can be imported formally, without using the unit in which it is fully declared.
% This enables making forward references to such classes and breaking circular dependencies amongst units.
% However, as soon as you wish to actually do something with an entity of this class type (such as
% access one of its fields, send a message to it, or use it to inherit from), the compiler requires the full definition
% of the class to be in scope.
sym_e_interprocgoto_into_init_final_code_not_allowed=05081_E_Gotos into initialization or finalization blocks of units are not allowed
% Gotos into initialization or finalization blockse of units are not allowed.
sym_e_external_class_name_mismatch1=05082_E_Invalid external name "$1" for formal class "$2"
sym_e_external_class_name_mismatch2=05083_E_Complete class definition with external name "$1" here
% When a class is declared using a formal external definition, the actual external
% definition (if any) must specify the same external name as the formal definition
% (since both definitions refer to the same actual class type).
sym_w_library_overload=05084_W_Possible library conflict: symbol "$1" from library "$2" also found in library "$3"
% Some OS do not have library specific namespaces, for those
% OS, the function declared as "external 'libname' name 'funcname'",
% the 'libname' part is only a hint, funcname might also be loaded
% by another library. This warning appears if 'funcname' is used twice
% with two different library names.
sym_e_duplicate_id_create_java_constructor=05085_E_Cannot add implicit constructor 'Create' because identifier already used by "$1"
% Java does not automatically add inherited constructors to child classes, so that they can be hidden.
% However, if a class does not explicitly declare at least one constructor, the compiler is
% required to add a public, parameterless constructor. In Java, constructors are nameless,
% but in FPC they are all called ``Create''. Therefore, if you do not add a constructor to
% a Java class and furthermore use the ``Create'' identifier for another entity (e.g., a field,
% or a parameterless method), the compiler cannot satisfy this requirement.
sym_e_no_matching_inherited_parameterless_constructor=05086_E_Cannot generate default constructor for class, because parent has no parameterless constructor
% Java does not automatically add inherited constructors to child classes, so that they can be hidden.
% However, if a class does not explicitly declare at least one constructor, the compiler is
% required to add a public, parameterless constructor. This compiler must then call
% the parameterless constructor from the parent class inside this added constructor.
% This is however impossible if the parent class does not declare such a constructor.
% In this case you must add a valid constructor yourself.
sym_d_adding_helper_for=05087_D_Adding helper for $1
% A helper for the mentioned type is added to the current scope
sym_e_param_list=05088_E_Found declaration: $1
% This message shows all overloaded declarations in case of an error.
sym_w_uninitialized_managed_local_variable=05089_W_Local variable "$1" of a managed type does not seem to be initialized
% This message is displayed if the compiler thinks that a variable will
% be used (i.e. it appears in the right-hand side of an expression) when it
% was not initialized first (i.e. appeared in the left-hand side of an
% assignment). Since the variable is managed, i. e. implicitly initialized by the compiler, this might be intended behaviour and
% does not necessarily mean that the code is wrong.
sym_w_uninitialized_managed_variable=05090_W_Variable "$1" of a managed type does not seem to be initialized
% This message is displayed if the compiler thinks that a variable will
% be used (i.e. it appears in the right-hand side of an expression) when it
% was not initialized first (i.e. appeared in the left-hand side of an
% assignment). Since the variable is managed, i. e. implicitly initialized by the compiler, this might be intended behaviour and
% does not necessarily mean that the code is wrong.
sym_h_uninitialized_managed_local_variable=05091_H_Local variable "$1" of a managed type does not seem to be initialized
% This message is displayed if the compiler thinks that a variable will
% be used (i.e. it appears in the right-hand side of an expression) when it
% was not initialized first (i.e. it did not appear in the left-hand side of an
% assignment). Since the variable is managed, i. e. implicitly initialized by the compiler, this might be intended behaviour and
% does not necessarily mean that the code is wrong.
sym_h_uninitialized_managed_variable=05092_H_Variable "$1" of a managed type does not seem to be initialized
% This message is displayed if the compiler thinks that a variable will
% be used (i.e. it appears in the right-hand side of an expression) when it
% was not initialized first (i.e. t did not appear in the left-hand side of an
% assignment). Since the variable is managed, i. e. implicitly initialized by the compiler, this might be intended behaviour and
% does not necessarily mean that the code is wrong.
sym_w_managed_function_result_uninitialized=05093_W_function result variable of a managed type does not seem to initialized
% This message is displayed if the compiler thinks that the function result
% variable will be used (i.e. it appears in the right-hand side of an expression)
% before it is initialized (i.e. before it appeared in the left-hand side of an
% assignment). Since the variable is managed, i. e. implicitly initialized by the compiler, this might be intended behaviour and
% does not necessarily mean that the code is wrong.
sym_h_managed_function_result_uninitialized=05094_H_Function result variable of a managed type does not seem to be initialized
% This message is displayed if the compiler thinks that the function result
% variable will be used (i.e. it appears in the right-hand side of an expression)
% before it is initialized (i.e. it appears in the left-hand side of an
% assignment). Since the variable is managed, i. e. implicitly initialized by the compiler, this might be intended behaviour and
% does not necessarily mean that the code is wrong.
sym_w_duplicate_id=05095_W_Duplicate identifier "$1"
% The identifier was already declared in an Objective-C category that's in the
% same scope as the current identifier. This is a warning instead of an error,
% because while this hides the identifier from the category, there are often
% many unused categories in scope.
% \end{description}
#
# Codegenerator
#
# 06049 is the last used one
#
% \section{Code generator messages}
% This section lists all messages that can be displayed if the code
% generator encounters an error condition.
% \begin{description}
cg_e_parasize_too_big=06009_E_Ukuran daftar parameter melebihi 65535 byte
% Prosesor I386 membatasi daftar parameter ke 65535 byte (instruksi \var{RET}
% menyebabkan ini)
cg_e_file_must_call_by_reference=06012_E_Tipe file harus parameter var
% Anda tidak bisa menetapkan file sebagai parameter nilai, misalnya ia harus
% selalu dideklarasikan parameter \var{var}.
cg_e_cant_use_far_pointer_there=06013_E_Penggunaan pointer far tidak dibolehkan di sini
% Free Pascal tidak mendukung pointer far, maka anda tidak bisa mengambil alamat
% sebuah ekspresi yang memiliki referensi far sebagai hasil. Konstruksi \var{mem}
% mempunyai referensi far sebagai hasil, maka kode berikut akan menghasilkan
% kesalahan ini:
% \begin{verbatim}
% var p : pointer;
% ...
% p:=@mem[a000:000];
% \end{verbatim}
cg_e_dont_call_exported_direct=06015_E_Fungsi yang dideklarasikan EXPORT tidak bisa dipanggil
% Tidak dipakai lagi.
cg_w_member_cd_call_from_method=06016_W_Kemungkinan tidak benar pemanggilan atas constructor atau destructor
% Kompilator mendeteksi bahwa constructor atau destructor dipanggil di dalam
% sebuah metode. Ini mungkin akan menimbulkan masalah, karena constructors/ 
% destructors memerlukan parameter saat masuk.
cg_n_inefficient_code=06017_N_Kode tidak efisien
% Pernyataan anda nampaknya diragukan oleh kompilator.
cg_w_unreachable_code=06018_W_Kode tidak bisa dicapai
% Anda menetapkan sebuah konstruksi yang tidak akan pernah dieksekusi. Contoh:
% \begin{verbatim}
% while false do
%   begin
%   {.. code ...}
%   end;
% \end{verbatim}
cg_e_cant_call_abstract_method=06020_E_Metode abstrak tidak bisa dipanggil secara langsung
% Anda tidak bisa memanggil metode abstrak secara langsung, sebaliknya anda
% harus memanggil metode anak penggantinya, karena metode abstrak tidak diterapkan.
cg_d_register_weight=06027_DL_Register $1 bobot $2 $3
% Pesan debug. Ditampilkan saat kompilator menganggap variabel untuk dipelihara
% dalam register.
cg_d_stackframe_omited=06029_DL_Bingkai stack diabaikan
% Beberapa prosedur/fungsi tidak memerlukan bingkai stack lengkap, maka ia
% diabaikan. Pesan ini akan ditampilkan saat saklar {-vd} dipakai.
cg_e_unable_inline_object_methods=06031_E_Metode obyek atau kelas tidak bisa inline.
% Anda tidak bisa mempunyai metode obyek inlined.
cg_e_unable_inline_procvar=06032_E_Pemanggilan procvar tidak dalam inline.
% Prosedur dengan pemanggilan variabel prosedural tidak bisa disejajarkan.
cg_e_no_code_for_inline_stored=06033_E_Tidak ada kode untuk prosedur inline tersimpan
% Kompilator tidak bisa menyimpan  kode untuk prosedur inline.
cg_e_can_access_element_zero=06035_E_Elemen nol dari ansi/wide- atau longstring tidak dapat diakses, sebaiknya gunakan (set)length
% Anda harus menggunakan \var{setlength} untuk menyetel panjang ansi/wide/longstring
% dan \var{length} untuk mendapatkan panjang dari tipe string tersebut
cg_e_cannot_call_cons_dest_inside_with=06037_E_Konstruktor atau Destruktor tidak bisa dipanggil di dalam klausul 'with'
% Di dalam klausul \var{with} anda tidak dapat memanggil sebuah konstruktor
% atau destruktor untuk obyek yang anda miliki dalam klausul \var{with}.
cg_e_cannot_call_message_direct=06038_E_Tidak bisa memanggil metode pengendali pesan secara langsung
% Pengendali metode pesan tidak bisa dipanggil secara langsung jika ia berisi
% argumen self eksplisit
cg_e_goto_inout_of_exception_block=06039_E_Lompat masuk atau keluar blok eksepsi
% Tidak dibolehkan untuk melompat masuk atau keluar dari blok eksepsi seperti \var{try..finally..end;}:
% \begin{verbatim}
% label 1;
%
% ...
%
% try
%    if not(final) then
%      goto 1;   // baris ini akan menimbulkan kesalahan
% finally
%   ...
% end;
% 1:
% ...
% \end{verbatim}
cg_e_control_flow_outside_finally=06040_E_Pernyataan alur kontrol tidak dibolehkan dalam blok finally
% Tidak dibolehkan untuk menggunakan pernyataan alur kontrol \var{break},
% \var{continue} dan \var{exit}
% di dalam pernyataan finally. Contoh berikut menampilkan masalah ini:
% \begin{verbatim}
% ...
%   try
%      p;
%   finally
%      ...
%      exit;  // Exit ini TIDAK diperbolehkan
%   end;
% ...
%
% \end{verbatim}
% Jika prosedur \var{p} memunculkan eksepsi blok finally dijalankan. Jika
% eksekusi mencapai exit, tidak jelas apa yang dilakukan:
% keluar prosedur, atau mencari pengendali eksepsi yang lain
cg_w_parasize_too_big=06041_W_Ukuran parameters melebihi batas cpu tertentu
% Ini menunjukan bahwa anda mendeklarasikan lebih dari 64K parameter, yang
% mungkin tidak didukung pada prosesor target.
cg_w_localsize_too_big=06042_W_Ukuran variabel lokal melebihi batas untuk cpu tertetnu
% Ini menunjukan bahwa anda mendeklarasikan lebih dari 32K varbael lokal, yang
% mungkin tidak didukung pada prosesor target.
cg_e_localsize_too_big=06043_E_Ukuran variabel lokal melebihi batas yang didukung
% Ini menunjukan bahwa anda mendeklarasikan lebih dari 32K variabel lokal, yang
% tidak didukung oleh prosesor ini.
cg_e_break_not_allowed=06044_E_BREAK tidak diperbolehkan
% Anda mencoba untuk menggunakan \var{break} di luar konstruksi loop.
cg_e_continue_not_allowed=06045_E_CONTINUE tidak diperbolehkan
% Anda mencoba untuk menggunakan \var{continue} di luar konstruksi loop.
cg_f_unknown_compilerproc=06046_F_Tidak dikenal compilerproc "$1". Periksa apakah anda menggunakan run time library yang benar.
% Kompilator mengharapkan bahwa runtime library berisi subrutin tertentu. Jika
% anda melihat kesalahan ini dan anda tidak mengubah kode runtime library, 
% nampaknya runtime library yang anda pakai tidak sesuai dengan kompilator. 
% Jika anda mengubah runtime library kesalahan ini berarti bahwa anda
% menghapus subrutin yang diperlukan kompilator untuk pemakaian internalnya.
cg_f_unknown_system_type=06047_F_Tidak bisa menemukan tipe sistem "$1". Periksa apakah anda menggunakan run time library yang benar.
% Kompilator mengharapkan bahwa runtime library berisi definisi tipe tertentu.
% Jika anda melihat kesalahan ini dan anda tidak mengubah kode runtime library, 
% nampaknya runtime library yang anda pakai tidak sesuai dengan kompilator. 
% Jika anda mengubah runtime library kesalahan ini berarti bahwa anda
% menghapus subrutin yang diperlukan kompilator untuk pemakaian internalnya.
cg_h_inherited_ignored=06048_H_Pemanggilan turunan ke metode abstrak diabaikan
% Pesan ini hanya muncul dalam mode Delphi ketika anda memanggil metode abstract
% dari kelas leluhur via \var{inherited;}. Pemanggilan in kemudian diabaikan.
% \end{description}
# EndOfTeX

#
# Assembler reader
#
# 07105 is the last used one
#
cg_e_goto_label_not_found=06049_E_Goto label "$1" not defined or optimized away
% The label used in the goto definition is not defined or optimized away by the
% unreachable code elemination.
cg_f_unknown_type_in_unit=06050_F_Cannot find type "$1" in unit "$2". Check if you use the correct run time library.
% The compiler expects that the runtime library contains certain type definitions. If you see this error
% and you didn't change the runtime library code, it's very likely that the runtime library
% you're using doesn't match the compiler in use. If you changed the runtime library this error means
% that you removed a type which the compiler needs for internal use.
cg_e_interprocedural_goto_only_to_outer_scope_allowed=06051_E_Interprocedural gotos are allowed only to outer subroutines
% Gotos between subroutines are only allowed if the goto jumps from an inner to an outer subroutine or
% from a subroutine to the main program
cg_e_labels_cannot_defined_outside_declaration_scope=06052_E_Label must be defined in the same scope as it is declared
% In ISO mode, labels must be defined in the same scope as they are declared.
cg_e_goto_across_procedures_with_exceptions_not_allowed=06053_E_Leaving procedures containing explicit or implicit exceptions frames using goto is not allowed
% Non-local gotos might not be used to leave procedures using exceptions either implicitly or explicitly. Procedures
% which use automated types like ansistrings or class constructurs are affected by this too.
cg_e_mod_only_defined_for_pos_quotient=06054_E_In ISO mode, the mod operator is defined only for positive quotient
% In ISO pascal, only positive values are allowed for the quotient: \var{n mod m} is only valid if \var{m>0}.
cg_d_autoinlining=06055_DL_Auto inlining: $1
% Due to auto inlining turned on, the compiler auto inlines this subroutine.
cg_e_function_not_support_by_selected_instruction_set=06056_E_The function used, is not supported by the selected instruction set: $1
% Some functions cannot be implemented efficiently for certain instruction sets, one example is fused multiply/add.
% To avoid very inefficient code, the compiler complains in this case, so either select another instruction set
% or replace the function call by alternative code
cg_f_max_units_reached=06057_F_Maximum number of units ($1) reached for the current target
% Depending of target architecture, the number of units is limited. This limit
% has been reached. A unit counts only if it contains initialization or finalization count.
%
% \end{description}
# EndOfTeX
#
# Assembler reader
#
# 07125 is the last used one
#
asmr_d_start_reading=07000_DL_Mulai penguraian assembler gaya $1
% Ini memberitahu anda bahwa blok assembler sedang diuraikan
asmr_d_finish_reading=07001_DL_Selesasi penguraian assembler gaya $1
% Ini memberitahu anda bahwa blok assembler sudah selesai.
asmr_e_none_label_contain_at=07002_E_Pola non-label berisi @
% Pembeda yang bukan label tidak bisa berisi sebuah @.
asmr_e_building_record_offset=07004_E_Kesalahan pembangunan ofset record
% Terjadi kesalahan saat membangun ofset struktur record/object,
% ini bisa terjadi ketika tidak ada field yang ditetapkan sama sekali
% atau pembeda field yang dipakai tidak dikenal.
asmr_e_offset_without_identifier=07005_E_OFFSET dipakai tanpa pembeda
% Abda hanya bisa menggunakan OFFSET dengan sebuah pembeda. Sintaks lain tidak
% didukung
asmr_e_type_without_identifier=07006_E_TYPE dipakai tanpa pembeda
% Anda hanya bisa menggunakan TYPE dengan sebuah pembeda. Sintaks lain tidak
% didukung
asmr_e_no_local_or_para_allowed=07007_E_Tidak bisa menggunakan variabel lokal atau parameters di sini
% Anda tidak dapat menggunakan variabel lokal atau parameter di sini, karena
% pengalamatan lokal dan parameter dikerjakan menggunakan bingkai register pointer
% agar alamat tidak bisa diperoleh secara langsung.
asmr_e_need_offset=07008_E_perlu memakai OFFSET di sini
% Anda perlu menggunakan OFFSET <id> di sini untuk mendapatkan alamat pengenal.
asmr_e_need_dollar=07009_E_perlu memakai $ di sini
% Anda perlu menggunakan $<id> di sini untuk mendapatkan alamat pengenal.
asmr_e_cant_have_multiple_relocatable_symbols=07010_E_Tidak bisa menggunakan multipel simbol relocatable
% Anda tidak bisa mempunyai lebih dari satu simbol relocatable (variabel/konstan type)
% dalam satu argumen.
asmr_e_only_add_relocatable_symbol=07011_E_Simbol relocatable hanya dapat ditambahkan
% Simbol relocatable (variabel/konstan type) tidak bisa dipakai dengan operator
% lain. Hanya tambahan yang dibolehkan.
asmr_e_invalid_constant_expression=07012_E_Ekspresi konstan tidak benar
% Ada kesalahan dalam ekspresi konstan.
asmr_e_relocatable_symbol_not_allowed=07013_E_Simbol relocatable tidak dibolehkan
% Anda tidak bisa menggunakan simbol relocatable (variabel/konstan type) di sini.
asmr_e_invalid_reference_syntax=07014_E_Sintaks referensi tidak benar
% Ada kesalahan dalam referensi.
asmr_e_local_para_unreachable=07015_E_Anda tidak dapat mencapai $1 dari kode itu
% Anda tidak dapat membaca secara langsung nilai variabel lokal atau parameter
% dari tingkat prosedur lebih tinggi dalam kode assembler (kecuali untuk kode
% assembler lokal tanpa parameter maupun local).
asmr_e_local_label_not_allowed_as_ref=07016_E_Simbol local/label tidak dibolehkan sebagai referensi
% Anda tidak bisa menggunakan simbol lokal/label sebagai referensi
asmr_e_wrong_base_index=07017_E_Penggunaan basis dan indeks register tidak benar
% Ada kesalahan dengan base dan index register, keduanya mungkin
% tidak benar
asmr_w_possible_object_field_bug=07018_W_Kemungkinan kesalahan dalam penanganan field obyek
% Field obyek atau kelas dapat dicapai secara langsung dalam mode normal atau 
% objfpc tapi mode TP dan Delphi memperlakukan nama field sebagai ofset sederhana.
asmr_e_wrong_scale_factor=07019_E_Faktor skala yang ditetapkan salah
% Faktor skala yang diberikan salah, hanya 1,2,4 dan 8 yang dibolehkan
asmr_e_multiple_index=07020_E_Penggunaan multipel indeks register
% Anda mencoba menggunakan lebih dari satu indeks register
asmr_e_invalid_operand_type=07021_E_Tipe operand tidak benar
% Tipe operand tidak sama dengan opcode yang dipakai
asmr_e_invalid_string_as_opcode_operand=07022_E_String sebagai opcode operand tidak benar: $1
% String yang ditetapkan sebagai operand tidak sama dengan opcode ini
asmr_w_CODE_and_DATA_not_supported=07023_W_@CODE dan @DATA tidak didukung
% @CODE dan @DATA tidak didukung dan diabaikan.
asmr_e_null_label_ref_not_allowed=07024_E_Referensi label null tidak dibolehkan
asmr_e_expr_zero_divide=07025_E_Pembagian dengan nol dalam evaluator asm
% Ada pembagian dengan nol dalam ekspresi konstan
asmr_e_expr_illegal=07026_E_Ekspresi tidak benar
% Ada ekspresi tidak benar dalam ekspresi konstan
asmr_e_escape_seq_ignored=07027_E_urutan escape diabaikan: $1
% Ada string bergaya-C, tapi urutan escape dalam string tidak dikenal,
% dan karenanya diabaikan
asmr_e_invalid_symbol_ref=07028_E_Referensi simbol tidak benar
asmr_w_fwait_emu_prob=07029_W_Fwait dapat menyebabkan masalah emulasi dengan emu387
asmr_w_fadd_to_faddp=07030_W_$1 tanpa operand diterjemahkan menjadi $1P
asmr_w_enter_not_supported_by_linux=07031_W_Instruksi ENTER tidak didukung oleh Linux kernel
% Instruksi ENTER dapat membuat kesalahan halaman stack yang tidak ditangkap
% dengan benar oleh pengendali halaman i386 Linux.
asmr_w_calling_overload_func=07032_W_Pemanggilan fungsi overload dalam assembler
% Ada panggilan ke metode yang di-overload dalam blok assembler, ini dapat
% menjadi tanda di sana ada masalah
asmr_e_unsupported_symbol_type=07033_E_Tipe simbol untuk operand tidak didukung
asmr_e_constant_out_of_bounds=07034_E_Nilai konstan di luar jangkauan
asmr_e_error_converting_decimal=07035_E_Kesalahan konversi desimal $1
% Nilai konstan desimal tidak memiliki sintaks yang benar
asmr_e_error_converting_octal=07036_E_Kesalahan konversi oktal $1
% Nilai konstan oktal tidak memiliki sintaks yang benar
asmr_e_error_converting_binary=07037_E_Kesalahan konversi biner $1
% Nilai konstan biner tidak memiliki sintaks yang benar
asmr_e_error_converting_hexadecimal=07038_E_Kesalahan konversi heksadesimal $1
% Nilai konstan heksadesimal tidak memiliki sintaks yang benar
asmr_h_direct_global_to_mangled=07039_H_$1 diterjemahkan menjadi $2
asmr_w_direct_global_is_overloaded_func=07040_W_$1 dikaitkan ke fungsi yang di-overload
asmr_e_cannot_use_SELF_outside_a_method=07041_E_Tidak bisa menggunakan SELF di luar metode
% Ada referensi ke simbol \var{self} sementara ia tidak diijinkan.
% \var{self} hanya bisa direferensi di dalam metode
asmr_e_cannot_use_OLDEBP_outside_nested_procedure=07042_E_Tidak bisa menggunakan OLDEBP di luar prosedure yang diulang
% Ada referensi ke simbol \var{oldebp} sementara ia tidak diijinkan.
% \var{oldebp} hanya bisa direferensi di dalam rutin yang diulang
asmr_e_void_function=07043_W_Prosedure tidak bisa mengembalikan setiap nilai dalam kode asm
% Mencoba untuk mengembalikan nilai saat dalam sebuah prosedur. Sebuah prosedur
% tidak mempunyai nilai balik
asmr_e_SEG_not_supported=07044_E_SEG tidak didukung
asmr_e_size_suffix_and_dest_dont_match=07045_E_Ukuran sufiks dan tujuan atau sumber tidak sama
% Ukuran register dan sufiks ukuran opcode tidak sama. Ini mungkin kesalahan
% dalam pernyataan assembler
asmr_w_size_suffix_and_dest_dont_match=07046_W_Ukuran sufiks dan tujuan atau sumber tidak sama
% Ukuran register dan sufiks ukuran opcode tidak sama. Ini mungkin kesalahan
% dalam pernyataan assembler
asmr_e_syntax_error=07047_E_Sintaks assembler salah
% Ada kesalahan dalam sintaks assembler
asmr_e_invalid_opcode_and_operand=07048_E_Kombinasi opcode dan operand tidak benar
% Opcode tidak bisa dipakai dengan tipe operand ini
asmr_e_syn_operand=07049_E_Sintaks assembler salah dalam operand
asmr_e_syn_constant=07050_E_Kesalahan sintaks assembler dalam konstan
asmr_e_invalid_string_expression=07051_E_Ekspresi String tidak benar
asmr_w_const32bit_for_address=07052_W_konstan dengan simbol $1 untuk alamat yang tidak pada pointer
% Ekspresi konstan mewakili alamat yang tidak sesuai ke dalam sebuah pointer.
% Alamat mungkin tidak benar
asmr_e_unknown_opcode=07053_E_Opcode $1 tidak dikenal
% Opcode ini tidak dikenal
asmr_e_invalid_or_missing_opcode=07054_E_Tidak benar atau opcode hilang
asmr_e_invalid_prefix_and_opcode=07055_E_Kombinasi prefiks dan opcode tidak benar: $1
asmr_e_invalid_override_and_opcode=07056_E_Kombinasi override dan opcode tidak benar: $1
asmr_e_too_many_operands=07057_E_Terlalu banyak operand pada baris
% Ada terlalu banyak operand untuk opcode ini. Periksa sintaks assembler
% anda
asmr_w_near_ignored=07058_W_NEAR diabaikan
asmr_w_far_ignored=07059_W_FAR diabaikan
asmr_e_dup_local_sym=07060_E_Duplikasi simbol lokal $1
asmr_e_unknown_local_sym=07061_E_Simbol lokal $1 tidak didefinisikan
asmr_e_unknown_label_identifier=07062_E_Pengenal label tidak dikenal $1
asmr_e_invalid_register=07063_E_Nama register tidak benar
% Ada nama register tidak dikenal yang dipakai sebagai operand.
asmr_e_invalid_fpu_register=07064_E_Nama register floating point tidak benar
% Ada nama register tidak dikenal yang dipakai sebagai operand.
asmr_w_modulo_not_supported=07066_W_Modulo tidak didukung
asmr_e_invalid_float_const=07067_E_Konstan floating point $1 tidak benar
% Konstan floating point yang dideklarasikan dalam blok assembler tidak
% benar.
asmr_e_invalid_float_expr=07068_E_Ekspresi floating point tidak benar
% Ekspresi floating point yang dideklarasikan dalam blok assembler tidak
% benar.
asmr_e_wrong_sym_type=07069_E_Tipe simbol salah
asmr_e_cannot_index_relative_var=07070_E_Tidak bisa mengindeks var lokal atau parameter dengan register
% Mencoba untuk mengindeks menggunakan basis register simbol yang sudah relatif
% bagi register. Ini tidak mungkin, dan mungkin akan membawa kerusakan.
asmr_e_invalid_seg_override=07071_E_Ekspresi penggantian segmen tidak benar
asmr_w_id_supposed_external=07072_W_Pengenal $1 dianggap eksternal
% Ada referensi ke simbol yang tidak didefinisikan. Ini tidak menghasilkan
% kesalahan, karena simbol mungkin eksternal, tapi dapat menyebabkan masalah
% saat waktu link jika simbol tidak didefinisikan dimanapun.
asmr_e_string_not_allowed_as_const=07073_E_String tidak diijinkan sebagai konstan
% String karakter tidak dibolehkan sebagai konstan.
asmr_e_no_var_type_specified=07074_Tidak ada tipe variabel yang ditetapkan
% Sintaks mengharapkan tipe pengenal setelah titik, tapi tidak menemukan
% apapun.
asmr_w_assembler_code_not_returned_to_text=07075_E_kode assembler tidak dikembaliken ke seksi text
% Ada direktif dalam blok assembler untuk mengubah seksi, tapi di sana
% ada kekuarangan untuk ke seksi text di akhir blok assembler. Ini dapat
% menyebabkan kesalahan selama waktu link.
asmr_e_not_directive_or_local_symbol=07076_E_Bukan direktif atau simbol lokal $1
% Simbol ini tidak dikenal.
asmr_w_using_defined_as_local=07077_E_Menggunakan nama yang didefinisikan sebagai label lokal
asmr_e_dollar_without_identifier=07078_E_Token dolar dipakai tanpa sebuah pengenal
% Ekspresi konstan memiliki pengenal yang tidak diawali dengan simbol $
asmr_w_32bit_const_for_address=07079_W_Konstan 32bit dibuat untuk alamat
% Konstan dipakai sebagai alamat. Ini mungkin kesalahan karena menggunakan
% alamat absolut yang mungkin tidak akan bekerja.
asmr_n_align_is_target_specific=07080_N_.align adalah spesifik target, gunakan .balign atau .p2align
% Menggunakan direktif .align adalah spesifik platform, dan artinya akan 
% berlainan dari satu platform ke yang lainnya.
asmr_e_cannot_access_field_directly_for_parameters=07081_E_Tidak bisa mengakses field secara langsung untuk parameter
% Anda harus mengambil parameter lebih duku ke dalam register dan kemudian
% mengakses field menggunakan register itu.
asmr_e_cannot_access_object_field_directly=07082_E_Tidak bisa mengakses field dari obyek/kelas secara langsung
% Anda harus mengambil pointer self lebih dulu ke dalam register dan mengakses
% field menggunakan register sebagai basis. Standarnya pointer self tersedia
% dalam register esi pada i386.
asmr_e_unable_to_determine_reference_size=07083_E_Tidak ada ukuran yang ditetapkan dan tidak bisa menentukan besar operand
% Anda harus menetapkan secara eksplisit ukuran untuk referensi, karena kompilator
% tidak bisa menentukan ukuran apa (byte, word, dword, dll) ia harus dipakai
% untuk referensi.
asmr_e_cannot_use_RESULT_here=07084_E_Tidak bisa menggunakan RESULT dalam fungsi ini
% Beberapa fungsi yang mengembalikan tipe kompleks tidak bisa menggunakan kata
% kunci \var{result}.
asmr_w_adding_explicit_args_fXX=07086_W_"$1" tanpa operand yang diterjemahkan ke dalam "$1 %st,%st(1)"
asmr_w_adding_explicit_first_arg_fXX=07087_W_"$1 %st(n)" diterjemahkan ke dalam "$1 %st,%st(n)"
asmr_w_adding_explicit_second_arg_fXX=07088_W_"$1 %st(n)" diterjemahkan ke dalam "$1 %st(n),%st"
asmr_e_invalid_char_smaller=07089_E_Char < tidak dibolehkan di sini
% Operator shift memerlukan karakter <<. Hanya salah satu dari karakter itu
% yang ditemukan.
asmr_e_invalid_char_greater=07090_E_Char > tidak dibolehkan di sini
% Operator shift memerlukan karakter >>. Hanya salah satu dari karakter itu
% yang ditemukan.
asmr_w_align_not_supported=07093_W_ALIGN tidak didukung
asmr_e_no_inc_and_dec_together=07094_E_Inc dan Dec tidak bisa bersamaan
% Mencoba untuk menggunakan increment dan decrement di dalam opcode yang sama
% pada 680x0. Ini tidak mungkin.
asmr_e_invalid_reg_list_in_movem=07095_E_reglist untuk movem tidak benar
% Mencoba untuk menggunakan opcode \var{movem} dengan register yang tidak
% benar untuk menyimpan dan mengembalikan.
asmr_e_invalid_reg_list_for_opcode=07096_E_Reglist tidak benar untuk opcode
asmr_e_higher_cpu_mode_required=07097_E_Mode cpu lebih tinggi diperlukan ($1)
% Mencoba untuk menggunakan instruksi yang tidak didukung dalam mode cpu saat
% ini. Gunakan pembuatan cpu lebih tinggi agar bisa menggunakan opcode ini
% dalam blok assembler anda
asmr_w_unable_to_determine_reference_size_using_dword=07098_W_Tidak ada ukuran yang ditetapkan dan tidak bisa menentukan besar operands, menggunakan DWORD sebagai standar
% Anda harus menetapkan secara eksplisit ukuran untuk referensi, karena
% kompilator tidak dapat menentukan ukuran apa (byte, word, dword, dll)
% yang harus dipakai untuk referensi. Peringatan ini hanya dipakai dalam mode
% Delphi di mana ia kembali menggunakan DWORD sebagai standar.
asmr_e_illegal_shifterop_syntax=07099_E_Kesalahan sintaks saat mencoba mengurai shifter operand
% Hanya ARM; ARM assembler mendukung apa yang disebut shifter operand. Sintaks
% yang dipakai bukan shifter operand yang benar. Contoh untuk operasi dengan shifter operand:
% \begin{verbatim}
% asm
%   orr     r2,r2,r2,lsl #8
% end;
% \end{verbatim}
asmr_e_packed_element=07100_E_Alamat komponen yang di-packed tidak pada batasan byte
% Komponen yang di-packed (field record dan elemen array) dapat dimulai pada bit
% mana saja di dalam sebuah byte. Pada CPU yang tidak mendukung memori 
% dialamatkan-bit (yang menyertakan semua CPU yang didukung oleh FPC saat ini) anda akan
% mendapatkan pesan kesalahan saat mencoba untuk mengindeks arrays dengan elemen
% yang ukurannya bukan multipel dari 8 bit. Hal yang sama untuk mengakses field record
% dengan alamat demikian dengan multipel dari 8 bit.
asmr_w_unable_to_determine_reference_size_using_byte=07101_W_Ukuran tidak ditetapkan dan tidak bisa menentukan besar dari operand, menggunakan BYTE sebagai standar
% Anda harus menetapkan secara eksplisit ukuran untuk referensi, karena
% kompilator tidak dapat menentukan ukuran apa (byte, word, dword, dll)
% yang harus dipakai untuk referensi. Peringatan ini hanya dipakai dalam mode
% Delphi di mana ia kembali menggunakan BYTE sebagai standar.
asmr_w_no_direct_ebp_for_parameter=07102_W_Penggunaan +offset(%ebp) untuk parameter di sini tidak benar
% Menggunakan referensi 8(%ebp) langsung untuk parameter fungsi/prosedur tidak
% benar jika parameter ada dalam register.
asmr_w_direct_ebp_for_parameter_regcall=07103_W_Penggunaan +offset(%ebp) tidak kompatibel dengan konvensi regcall
% Menggunakan referensi 8(%ebp) langsung untuk parameter fungsi/prosedur tidak
% benar jika parameter ada dalam register.
asmr_w_direct_ebp_neg_offset=07104_W_Penggunaan -offset(%ebp) tidak direkomendasikan untuk akses variabel lokal
% Menggunakan -8(%ebp) untuk mengakses variabel lokal tidak direkomendasikan
asmr_w_direct_esp_neg_offset=07105_W_Penggunaan -offset(%esp), akses dapat menyebabkan kerusakan atau nilai bisa hilang
% Menggunakan -8(%esp) untuk mengakses stack lokal tidak direkomendasikan karena
% bagian stack ini dapat diganti oleh setiap panggilan fungsi atau interrupt.
asmr_e_no_vmtoffset_possible=07106_E_VMTOffset harus dipakai dalam kombinasi dengan metode virtual, dan "$1" bukanlah virtual
%
#
# Assembler/binary writers
#
# 08018 is the last used one
#
asmr_e_need_pic_ref=07107_E_Generating PIC, but reference is not PIC-safe
% The compiler has been configured to generate position-independent code
% (PIC), but there are position-dependent references in the current
% handwritten assembler instruction.
asmr_e_mixing_regtypes=07108_E_All registers in a register set must be of the same kind and width
% Instructions on the ARM architecture that take a register set as argument require that all registers
% in this set are of the same kind (e.g., integer, vfp) and width (e.g., single precision, double precision).
asmr_e_empty_regset=07109_E_A register set cannot be empty
% Instructions on the ARM architecture that take a register set as argument require that such a set
% contains at least one register.
asmr_w_useless_got_for_local=07110_W_@GOTPCREL is useless and potentially dangerous for local symbols
% The use of @GOTPCREL supposes an extra indirection that is
% not present if the symbol is local, which might lead to wrong assembler code
asmr_w_general_segment_with_constant=07111_W_Constant with general purpose segment register
% General purpose register should not have constant offsets
% as OS memory allocation might not be compatible with that.
asmr_e_bad_seh_directive_offset=07112_E_Invalid offset value for $1
% Win64 SEH directives have certain restrictions on possible offset values, e.g. they should
% be positive and have 3 or 4 low bits clear.
asmr_e_bad_seh_directive_register=07113_E_Invalid register for $1
% Win64 SEH directives accept only 64-bit integer registers or XMM registers.
asmr_e_seh_in_pure_asm_only=07114_E_SEH directives are allowed only in pure assembler procedures
% Win64 SEH directives are allowed only in pure assembler procedures, not in assembler
% blocks of regular procedures.
asmr_e_unsupported_directive=07115_E_Directive "$1" is not supported for the current target
asmr_e_complex_function_result_location=07116_E_This function's result location cannot be encoded directly in a single operand when "nostackframe" is used
% Functions declared with the \var{nostackframe} modifier do not have a stack frame, and hence
% do not have a local variable to hold the temporary function result.  Accesses to the function
% result in such routines directly go to the registers or memory location that will be used
% to return their result to the caller.  In some cases this result may be spread over multiple
% registers, in which case it is not possible to access the location via its symbolic name.
% You have to directly use the appropriate register names in this case.
asmr_e_wrong_gotpcrel_intel_syntax=07117_E_GOTPCREL references in Intel assembler syntax cannot contain a base or index register, and their offset must 0.
% The syntax for a gotpcrel PIC memory expression in Intel assembler syntax is
% \verb*[global_symbol wrt ..gotpcrel]*
asmr_e_no_gotpcrel_support=07118_E_The current target does not support GOTPCREL relocations
% Not all targets support position-independent code using a global offset table.
% Use a different way to access symbols in a position-indepent way in these cases.
asmr_w_global_access_without_got=07119_W_Exported/global symbols should be accessed via the GOT
% Global symbols (symbols from the unit interface, or defined in a program
% or library) should be accessed via the GOT when generating position-indepent code.
asmr_w_check_mem_operand_size=07120_W_Check size of memory operand "$1"
% The size of memory operand is possible invalid. This is
% probably an error in the assembler statement
asmr_w_check_mem_operand_size3=07121_W_Check size of memory operand "$1: memory-operand-size is $2 bits, but expected [$3 bits]"
% The size of memory operand is possible invalid. This is
% probably an error in the assembler statement
asmr_w_check_mem_operand_size_offset=07122_W_Check size of memory operand "$1: memory-operand-size is $2 bits, but expected [$3 bits + $4 byte offset]"
% The size of memory operand is possible invalid. This is
% probably an error in the assembler statement
asmr_w_check_mem_operand_negative_offset=07123_W_Check "$1: offset of memory operand is negative "$2 byte"
% The offset of memory operand is possible invalid. This is
% probably an error in the assembler statement
asmr_w_check_mem_operand_automap_multiple_size=07124_W_Check "$1: size of memory operand is empty, but es exists different definitions of the memory size =>> map to $2 (smallest option)"
% The size of memory operand is empty and we have different definitions of possible memory sizes. Check it!
asmr_e_invalid_ref_register=07125_E_Invalid register used in memory reference expression: "$1"
% FPU, vector and sometimes integer registers cannot be used in memory reference
% expressions, due to limitations of the cpu architecture or simple because
% it is not meaningful.

#
# Assembler/binary writers
#
# 08029 is the last used one
#
asmw_f_too_many_asm_files=08000_F_Terlalu banyak file assembler
% Dengan menghidupkan smartlinking, ada terlalu banyak file assembler yang dibuat.
% Matikan smartlinking.
asmw_f_assembler_output_not_supported=08001_F_Output assembler yang dipilih tidak didukung
asmw_f_comp_not_supported=08002_F_Comp tidak didukung
asmw_f_direct_not_supported=08003_F_Direct tidak mendukung penulis biner
% Mode assembler direct tidak mendukung penulis biner.
asmw_e_alloc_data_only_in_bss=08004_E_Alikasi data hanya dibolehkan dalam seksi bss
asmw_f_no_binary_writer_selected=08005_F_Tidak ada penulis biner yang dipilih
asmw_e_opcode_not_in_table=08006_E_Asm: Opcode $1 tidak dalam tabel
asmw_e_invalid_opcode_and_operands=08007_E_Asm: $1 kombinasi opcode dan operan tidak benar
asmw_e_16bit_not_supported=08008_E_Asm: referensi 16 Bit tidak didukung
asmw_e_invalid_effective_address=08009_E_Asm: Alamat efektif tidak benar
asmw_e_immediate_or_reference_expected=08010_E_Asm: Immediate atau referensi diharapkan
asmw_e_value_exceeds_bounds=08011_E_Asm: $1 nilai melebihi batasan $2
asmw_e_short_jmp_out_of_range=08012_E_Asm: Lompat pendek di luar jangkauan $1
asmw_e_undefined_label=08013_E_Asm: Label tidak didefinisikan $1
asmw_e_comp_not_supported=08014_E_Asm: Tipe Comp tidak didukung untuk target ini
asmw_e_extended_not_supported=08015_E_Asm: Tipe Extended tidak didukung untuk target ini
asmw_e_duplicate_label=08016_E_Asm: Duplikasi label $1
asmw_e_redefined_label=08017_E_Asm: Redefinisi label $1
asmw_e_first_defined_label=08018_E_Asm: Pertama didefinisikan di sini
asmw_e_invalid_register=08019_E_Asm: register tidak benar $1
asmw_e_16bit_32bit_not_supported=08020_E_Asm: Referensi 16 atau 32 Bit tidak didukung
asmw_e_64bit_not_supported=08021_E_Asm: 64 Bit operand tidak didukung

#
# Menjalankan linker/assembler
#
# 09034 is the last used one
#
# BeginOfTeX
%
% \section{Kesalahan tahap assembling/linking}
% Seksi ini mendaftarkan kesalahan yang terjadi saat kompilator memproses
% baris perintah atau menangani file konfigurasi.
% \begin{description}
asmw_e_bad_reg_with_rex=08022_E_Asm: AH,BH,CH or DH cannot be used in an instruction requiring REX prefix
% x86_64 only: instruction encoding of this platform does not allow using
% 8086 high byte registers (AH,BH,CH or DH) together with REX prefix in a single instruction.
% The REX prefix is required whenever the instruction operand size is 64 bits, or
% when it uses one of extended x86_64 registers (R8-R15 or XMM8-XMM15).
asmw_e_missing_endprologue=08023_E_Missing .seh_endprologue directive
% x86_64-win64 only: Normally, SEH directives are handled internally by compiler.
% However, in pure assembler procedures .seh_endprologue directive is required
% if other SEH directives are present.
asmw_e_prologue_too_large=08024_E_Function prologue exceeds 255 bytes
% x86_64-win64: .seh_prologue directive must be placed within 255 bytes from function start.
asmw_e_handlerdata_no_handler=08025_E_.seh_handlerdata directive without preceding .seh_handler
% x86_64-win64: If .seh_handlerdata directive is used, then a .seh_handler directive must be
% present earlier in the same function.
asmw_f_too_many_relocations=08026_F_Relocation count for section $1 exceeds 65535
% Legacy COFF targets limit number of relocations per section to 65535 because they use a 2-byte field
% to store the relocation count. Targets using newer PECOFF format do not have this limitation.
asmw_w_changing_bind_type=08027_N_Change of bind type of symbol $1 from $2 to $3 after use
asmw_h_changing_bind_type=08028_H_Change of bind type of symbol $1 from $2 to $3 after use
% An assembler symbol bind type has been altered after use, which can lead to wrong code.
% First version is reserved for changig to local label, which is the most probable cause
% of wrong code generation, but currently set to Note level as it appears inside
% the compiler compilation.
asmw_e_32bit_not_supported=08029_E_Asm: 32 Bit references not supported
#
# Executing linker/assembler
#
# 09035 is the last used one
#
# BeginOfTeX
%
% \section{Errors of assembling/linking stage}
% This section lists errors that occur when the compiler is processing the
% command line or handling the configuration files.
% \begin{description}
exec_w_source_os_redefined=09000_W_Sistem operasi sumber didefinisikan ulang
% Sistem operasi sumber didefinisikan ulang.
exec_i_assembling_pipe=09001_I_Assembling (pipe) $1
% Meng-assembling menggunakan pipa ke assembler eksternal.
exec_d_cant_create_asmfile=09002_E_Tidak bisa membuat file assembler: $1
% File yang disebutkan tadi tidak bisa dibuat. Periksa apakah anda memiliki
% perijinan akses untuk membuat file ini
exec_e_cant_create_objectfile=09003_E_Tidak bisa membuat file obyek: $1
% File yang disebutkan tadi tidak bisa dibuat. Periksa apakah anda memiliki
% perijinan akses untuk membuat file ini
exec_e_cant_create_archivefile=09004_E_Tidak bisa membuat file arsip: $1
% File yang disebutkan tadi tidak bisa dibuat. Periksa apakah anda memiliki
% perijinan akses untuk membuat file ini
exec_e_assembler_not_found=09005_E_Assembler $1 tidak ditemukan, beralih ke assmembling eksternal
% Program assembler tidak ditemukan. Kompilator akan menghasilkannaskah yang 
% dapat dipakai untuk meng-assemble dan me-link program.
exec_t_using_assembler=09006_T_Menggunakan assembler: $1
% Pesan informasi yang mengatakan assembler mana yang sedang dipakai.
exec_e_error_while_assembling=09007_E_Kesaslahan saat meng-assembling exitcode $1
% Ada kesalahan saat meng-assembling file menggunakan assembler eksternal.
% Lihat dokumentasi piranti assembler guna mencari informasi lebih jauh
% mengenai kesalahan ini.
exec_e_cant_call_assembler=09008_E_Tidak bisa memanggil assembler, kesalahan $1 beralih ke assembling eksternal
% Kesalahan terjadi saat memanggil assembler eksternal, kompilator akan menghasilkan
% naskah yang dapat dipakai untuk meng-assemble dan link program.
exec_i_assembling=09009_I_Pemasangan $1
% Pesan informasional yang menyatakan file yang sedang dipasang.
exec_i_assembling_smart=09010_I_Memasang dengan smartlinking $1
% Pesan informasional yang menyatakan file mana yang sedang dipasang menggunakan smartlinking.
exec_w_objfile_not_found=09011_W_Obyek $1 tidak ditemukan, Linking bisa gagal !
% Salah satu file obyek hilang, dan linking kemungkinan akan gagal.
% Periksa path anda.
exec_w_libfile_not_found=09012_W_Librari $1 tidak ditemukan, Linking bisa gagal !
% Salah satu file librari hilang, dan linking kemungkinan akan gagal.
% Periksa path anda.
exec_e_error_while_linking=09013_E_Kesalahan saat me-link
% Kesalahan umum saat melakukan linking.
exec_e_cant_call_linker=09014_E_Tidak bisa memanggil linker, beralih ke eksternal linking
% Kesalahan terjadi saat memanggil linker eksternal, kompulator akan membuat naskah
% yang bisa dipakai untuk memasang dan me-link program.
exec_i_linking=09015_I_Linking $1
% Pesan informasional, menampilkan program atau librari mana yang sedang di-link.
exec_e_util_not_found=09016_E_Util $1 tidak ditemukan, beralih ke eksternal linking
% Piranti eksternal tidak ditemukan, kompilator akan membuat naskah yang
% dapat dipakai untuk memasang dan me-link atau memproses akhir program.
exec_t_using_util=09017_T_Menggunakan util $1
% Pesan informasional, menampilkan program eksternal mana (biasanya pemroses akhir) yang dipakai.
exec_e_exe_not_supported=09018_E_Pembuatan Executable tidak didukung
% Pembuatan program executable tidak didukung untuk platform ini, karena belum
% diimplementasikan dalam kompilator.
exec_e_dll_not_supported=09019_E_Pembuatan Librari Dinamis/Berbagai tidak didukung
% Pembuatan dynamically loadable libraries tidak didukung untuk platform ini, 
% karena belum diimplementasikan dalam kompilator.
exec_e_static_lib_not_supported=09035_E_Creation of Static Libraries not supported
% Creating static libraries is not supported for this platform, because it was
% not yet implemented in the compiler.
exec_i_closing_script=09020_I_Menutup naskah $1
% Pesan informasional yang tampil saat naskah linking assembling eskternal selesai.
exec_e_res_not_found=09021_E_kompilator resource tidak ditemukan, beralih ke mode eksternal
% Kompilator resource eksternal tidak ditemukan, kompilator akan membuat naskah yang dapat 
% dipakai untuk memasang, mengompilasi resources dan me-link atau memproses akhir program.
exec_i_compilingresource=09022_I_Mengompilasi resource $1
% Pesan informasional, menampilkan resource mana yang sedang dikompilasi.
exec_t_unit_not_static_linkable_switch_to_smart=09023_T_unit $1 tidak bisa di-link secara statis, beralih ke smart linking
% Link statis diminta, tapi unit yang dipakai bukan yang bisa di-link secara statis.
exec_t_unit_not_smart_linkable_switch_to_static=09024_T_unit $1 tidak bisa di-link smart, beralih ke linking statis
% Smart linking diminta, tapi unit yang dipakai bukan yang bisa di-link samart.
exec_t_unit_not_shared_linkable_switch_to_static=09025_T_unit $1 tidak bisa di-link berbagi, berlaih ke linking statis
% Link berbagi yang diminta, tapi unit yang dipakai bukan yang bisa berbagi link.
exec_e_unit_not_smart_or_static_linkable=09026_E_unit $1 tidak bisa di-link smart atau statis
% Link smart atau statis diminta, tapi unit yang dipakai tidak bisa digunakan untuk keduanya.
exec_e_unit_not_shared_or_static_linkable=09027_E_unit $1 tidak bisa di-link berbagi ataupun statis
% Link berbagi atau statis yang diminta, tapi unit bukan yang bisa melakukan keduanya.
exec_d_resbin_params=09028_D_Memanggil kompilator resource "$1" dengan "$2" sebagai baris perintah
% Pesan informasional yang menampilkan baris perintah mana yang dipakai untuk kompilator resource.
%\end{description}
# EndOfTeX

#
# Informasi Executable
#
# BeginOfTeX
% \section{Pesan informasi executable.}
% Seksi ini mendaftarkan semua pesan yang dikeluarkan kompilator saat program
% executable dihasilkan, dan hanya ketika linker internal yang dipakai.
% \begin{description}
exec_e_error_while_compiling_resources=09029_E_Error while compiling resources
% The resource compiler or converter returned an error.
exec_e_cant_call_resource_compiler=09030_E_Can't call the resource compiler "$1", switching to external mode
% An error occurred when calling a resource compiler. The compiler will produce
% a script that can be used to assemble, compile resources and link or
% postprocess the program.
exec_e_cant_open_resource_file=09031_E_Can't open resource file "$1"
% An error occurred resource file cannot be opened.
exec_e_cant_write_resource_file=09032_E_Can't write resource file "$1"
% An error occurred resource file cannot be written.
exec_n_backquote_cat_file_not_found=09033_N_File "$1" not found for backquoted cat command
% The compiler did not find the file that should be expanded into linker parameters
exec_w_init_file_not_found=09034_W_"$1" not found, this will probably cause a linking failure
% The compiler adds certain startup code files to the linker only when they are found.
% If they are not found, they are not added and this might cause a linking failure.
%
%\end{description}
# EndOfTeX

#
# Executable information
#
# 09134 is the last used one
#
# BeginOfTeX
% \section{Executable information messages.}
% This section lists all messages that the compiler emits when an executable program is produced,
% and only when the internal linker is used.
% \begin{description}
execinfo_f_cant_process_executable=09128_F_Tidak dapat memproses akhir executable $1
% Kesalahan fatal saat kompilator tidak bisa memproses akhir sebuah executable.
execinfo_f_cant_open_executable=09129_F_Tidak bisa membuka executable $1
% Kesalahan fatal saat kompilator tidak bisa membuka file untuk executable.
execinfo_x_codesize=09130_X_Besar Kode: $1 byte
% Pesan informasional yang menampilkan ukuran seksi kode yang dihasilkan.
execinfo_x_initdatasize=09131_X_Besar data diinisialisasi: $1 byte
% Pesan informasional yang menampilkan ukuran seksi data diinisialisasi.
execinfo_x_uninitdatasize=09132_X_Besar data tidak diinisialisasi: $1 byte
% Pesan informasional yang menampilkan ukuran seksi data yang tidak diinisialisasi.
execinfo_x_stackreserve=09133_X_Ruang stack terpakai: $1 byte
% Pesan informasional yang menampilkan ukuran stack yang dipakai kompilator untuk executable.
execinfo_x_stackcommit=09134_X_Ruang stack dikomit: $1 byte
% Pesan informasional yang menampilkan ukuran stack yang dikomit oleh kompilator untuk executable.
%\end{description}
# EndOfTeX

#
# Pengambilan Unit
#
# 10041 is the last used one
#
# BeginOfTeX
% \section{Pesan pengambilan unit.}
% Seksi ini mendaftarkan semua pesan yang dapat terjadi saat kompilator
% menggambil unit dari disk ke dalam memori. Banyak dari pesan ini berupa
% pesan informasional.
% \begin{description}
link_f_executable_too_big=09200_F_Executable image size is too big for $1 target.
% Fatal error when resulting executable is too big.
link_w_32bit_absolute_reloc=09201_W_Object file "$1" contains 32-bit absolute relocation to symbol "$2".
% Warning when 64-bit object file contains 32-bit absolute relocations.
% In such case an executable image can be loaded into lower 4Gb of
% address space only.
%\end{description}
# EndOfTeX

#
# Unit loading
#
# 10062 is the last used one
#
# BeginOfTeX
% \section{Unit loading messages.}
% This section lists all messages that can occur when the compiler is
% loading a unit from disk into memory. Many of these messages are
% informational messages.
% \begin{description}
unit_t_unitsearch=10000_T_Pencarian unit: $1
% Ketika anda menggunakan saklar \var{-vt}, kompilator memberitahu anda ke mana
% ia mencoba mencari file unit.
unit_t_ppu_loading=10001_T_Pengambilan PPU $1
% Saat saklar \var{-vt} dipakai, kompilator memberitahu anda
% unit apa yang diambil.
unit_u_ppu_name=10002_U_Nama PPU: $1
% Saat anda menggunakan flag \var{-vu}, nama unit ditampilkan.
unit_u_ppu_flags=10003_U_Flag PPU: $1
% Saat anda menggunakan flag \var{-vu}, flag unit ditampilkan.
unit_u_ppu_crc=10004_U_Crc PPU: $1
% Saat anda memakai flag \var{-vu}, pemeriksaan CRC unit ditampilkan.
unit_u_ppu_time=10005_U_Waktu PPU: $1
% Saat anda memakai flag \var{-vu}, waktu unit dikompilasi ditampilkan.
unit_u_ppu_file_too_short=10006_U_File PPU terlalu pendek
% File ppu terlalu pendek, tidak semua deklarasi ada di sana.
unit_u_ppu_invalid_header=10007_U_Header PPU tidak benar (tanpa PPU di awal)
% File unit berisi tiga byte pertama kode ascii \var{PPU}
unit_u_ppu_invalid_version=10008_U_Versi PPU tidak benar $1
% File unit ini dikompilasi dengan versi kompilator berbeda, dan
% tidak bisa dibaca.
unit_u_ppu_invalid_processor=10009_U_PPU dikompilasi untuk prosesor lain
% File unit dikompilasi untuk tipe prosesor berbeda, dan
% tidak bisa dibaca
unit_u_ppu_invalid_target=10010_U_PPU dikompilasi untuk target lain
% File unit dikompilasi untuk target berbeda, dan
% tidak bisa dibaca
unit_u_ppu_source=10011_U_Sumber PPU: $1
% Saat anda memakai flag \var{-vu}, nama file sumber ditampilkan.
unit_u_ppu_write=10012_U_Menuliskan $1
% Saat anda menetapkan saklar \var{-vu}, kompilator akan memberitahu anda
% di mana ia menulis file unit.
unit_f_ppu_cannot_write=10013_F_Tidak bisa Menulis File-PPU
% Kesalahan terjadi saat menuliskan file unit.
unit_f_ppu_read_error=10014_F_Kesalahan membaca File-PPU
% Ini berarti bahwa file unit sudah rusak, dan berisi informasi tidak benar.
% Diperlukan rekompilasi.
unit_f_ppu_read_unexpected_end=10015_F_akhir File-PPU diharapkan
% Akhir file tidak diharapkan. Ini berarti bahwa file PPU 
% rusak.
unit_f_ppu_invalid_entry=10016_F_Entri File-PPU tidak benar: $1
% Unit yang coba dibaca kompilator rusak, atau dibuat dengan versi
% kompilator lebih baru.
unit_f_ppu_dbx_count_problem=10017_F_Masalah jumlah PPU Dbx
% Ada ketidak konsistenan dalam informasi debug pada unit.
unit_e_illegal_unit_name=10018_E_Nama unit tidak benar: $1
% Nama unit tidak sama dengan nama file.
unit_f_too_much_units=10019_F_Terlalu banyak unit
% \fpc mempunyai batasan 1024 unit dalam sebuah program. Anda dapat mengubah
% perliaku ini dengan mengubah konstan \var{maxunits} dalam file \file{fmodule.pas}
% pada kompilator, dan mengompilasi ulang kompilator.
unit_f_circular_unit_reference=10020_F_Referensi unit berputar antara $1 dan $2
% Dua unit saling menggunakan bagian interface. Ini hanya dibolehkan dalam
% bagian \var{implementation}. Setidaknya satu unit harus berisi satu yang
% lain dalam seksi \var{implementation}.
unit_f_cant_compile_unit=10021_F_Tidak bisa mengompilasi unit $1, tidak ada sumber tersedia
% Unit ditemukan yang memerlukan kompilasi ulang, tapi tidak ada file sumber
% yang tersedia.
unit_f_cant_find_ppu=10022_F_Tidak bisa menemukan unit $1 yang dipakai oleh $2
% Anda mencoba untuk memakai sebuah unit di mana file PPU tidak ditemukan oleh
% kompilator. Periksas file konfigurasi anda untuk path unit
unit_w_unit_name_error=10023_W_Unit $1 tidak ditemukan tapi $2 ada
% Pesan kesalahan ini tidak dipakai lagi.
unit_f_unit_name_error=10024_F_Unit $1 dicari tapi $2 yang ditemukan
% Pemotongan Dos pada 8 huruf untuk file unit PPU dapat membawa
% masalah saat nama unit lebih panjang dari 8 huruf.
unit_w_switch_us_missed=10025_W_Mengompilasi unit sistem memerlukan saklar -Us
% Ketika mengompilasi ulang unit sistem (ia memerlukan perlakuan khusus),
% \var{-Us} harus ditetapkan.
unit_f_errors_in_unit=10026_F_Ada $1 kesalahan dalam mengompilasi modul, dihentikan
% Saat kompilator menemukan sebuah kesalahan fatal atau terlalu banyak kesalahan
% dalam sebuah modul maka ia berhenti dengan pesan ini.
unit_u_load_unit=10027_U_Diambil dari $1 ($2) unit $3
% Saat anda memakai flag \var{-vu}, yang mana unit diambil dari unit itu
% ditampilkan.
unit_u_recompile_crc_change=10028_U_Rekompilasi $1, checksum berubah untuk $2
% Unit dikompilasi ulang karena checksum unit di mana ia bergantung padanya
% sudah berubah.
unit_u_recompile_source_found_alone=10029_U_Rekompilasi $1, hanya ditemukan sumber
% Saat anda menggunakan flag \var{-vu}, pesan ini memberitahu anda mengapa
% unit saat ini dikompilasi ulang.
unit_u_recompile_staticlib_is_older=10030_U_Rekompilasi unit, lib statis lebih lama dari ppufile
% Saat anda menggunakan flag \var{-vu}, kompilator memperingatkan bila unit 
% librari statis lebih lama dari file unit itu sendiri.
unit_u_recompile_sharedlib_is_older=10031_U_Rekompilasi unit, lib berbagi lebih lama dari ppufile
% Saat anda menggunakan flag \var{-vu}, kompilator memperingatkan bila unit 
% librari berbagi lebih lama dari file unit itu sendiri.
unit_u_recompile_obj_and_asm_older=10032_U_Rekompilasi unit, obj dan asm lebih lama dari ppufile
% Saat anda menggunakan flag \var{-vu}, kompilator memperingatkan bila assembler
% atau file obyek pada unit lebih lama dari file unit itu sendiri.
unit_u_recompile_obj_older_than_asm=10033_U_Rekompilasi unit, obj lebih lama dari asm
% Saat anda menggunakan flag \var{-vu}, kompilator memperingatkan bila file
% assembler pada unit lebih lama dari file obyek dalam unit.
unit_u_parsing_interface=10034_U_Menguraikan interface pada $1
% Saat anda menggunakan flag \var{-vu}, kompilator memperingatkan bahwa ia
% mulai menguraikan bagian interface pada unit
unit_u_parsing_implementation=10035_U_Menguraikan implementation pada $1
% Saat anda menggunakan flag \var{-vu}, kompilator memperingatkan bahwa ia
% mulai menguraikan bagian implementation pada unit
unit_u_second_load_unit=10036_U_Pengambilan kedua untuk unit $1
% Saat anda menggunakan flag \var{-vu}, kompilator memperingatkan bahwa ia
% mulai mengompilasi ulang unit untuk kedua kali. Ini dapat terjadi dengan
% unit interdependensi.
unit_u_check_time=10037_U_Pemeriksaan file PPU $1 waktu $2
% Saat anda menggunakan flag \var{-vu}, kompilator menampilkan nama file dan
% tanggal serta waktu rekompilasi file diandalkan
### Dua pesan kesalahan berikut saat ini dimatikan.
#unit_h_cond_not_set_in_last_compile=10038_H_Kondisional $1 tidak disetel di awal dalam kompilasi terkahir $2
#% Saat rekompilasi unit diperlukan, kompilator akan memeriksa kondisional
#% yang sama yang disetel untuk rekompilasi. Kompilator sudah menemukan
#% kondisional yang saat ini didefinisikan, tapi tidak dipakai terakhir kali
#% unit direkompilasi.
#unit_h_cond_set_in_last_compile=10039_H_Kondisional $1 disetel di awal dalam kompilasi terakhir $2
#% Saat rekompilasi unit memerlukan kompilator memeriksa bahwa kondisional
#% yang sama disetel untuk rekompilasi. Kompilator sudah menemukan sebuah
#% kondisional yang dipakai terakhir kali unit dikompilasi, tetapi
#% kondisional saat ini tidak didefinisikan.
unit_w_cant_compile_unit_with_changed_incfile=10040_W_Tidak bisa merekompilasi unit $1, tapi ditemukan file include yang dimodifikasi
% Sebuah unit ditemukan di mana file include sudah diubah, tapi
% beberapa file sumber tidak ditemukan, maka rekompilasi tidak memungkinkan.
unit_u_source_modified=10041_U_File $1 lebih baru daripada file PPU $2
% File sumber dimodifikasi untuk unit yang ditemukan kompilator.
unit_u_ppu_invalid_fpumode=10042_U_Mencoba menggunakan unit yang dikompilasi dengan mode FPU berbeda
% Mencoba untuk mengompilasi kode sementara menggunakan unit yang tidak 
% dikompilasi dengan mode format pecahan yang sama. Baik semua kode harus
% dikompilasi dengan emulasi FPU on ataupun dengan emulasi FPU off.
unit_u_loading_interface_units=10043_U_Mengambil unit interface dari $1
% Saat anda menggunakan flag \var{-vu}, kompilator meperingatkan bahwa ia
% mulai mengambil unit yang didefinisikan dalam bagian interface pada unit.
unit_u_loading_implementation_units=10044_U_Mengambil implementation unit dari $1
% Saat anda menggunakan flag \var{-vu}, kompilator meperingatkan bahwa ia
% mulai mengambil unit yang didefinisikan dalam bagian implementation pada unit.
unit_u_interface_crc_changed=10045_U_CRC Interface berubah untuk unit $1
% Saat anda menggunakan flag \var{-vu}, kompilator meperingatkan bahwa
% CRC yang dihitung untuk interface sudah berubah setelah interface
% diuraikan.
unit_u_implementation_crc_changed=10046_U_CRC Implementation berubah untuk unit $1
% Saat anda menggunakan flag \var{-vu}, kompilator meperingatkan bahwa
% CRC yang dihitung sudah diubah setelah implementation
% diuraikan.
unit_u_finished_compiling=10047_U_Selesai mengompilasi unit $1
% Saat anda menggunakan flag \var{-vu}, kompilator meperingatkan bahwa ia
% selesai mengompilasi unit.
unit_u_add_depend_to=10048_U_Menambah dependensi $1 ke $2
% Saat anda menggunakan flag \var{-vu}, kompilator meperingatkan bahwa ia
% sudah menambahkan dependensi antara dua unit.
unit_u_no_reload_is_caller=10049_U_Tidak mengambil ulang, pemanggil: $1
% Saat anda menggunakan flag \var{-vu}, kompilator meperingatkan bahwa ia
% tidak akan mengambil ulang unit karena ia adalah unit yang ingin
% mengambil unit ini
unit_u_no_reload_in_second_compile=10050_U_No reload, already in second compile: $1
% Saat anda menggunakan flag \var{-vu}, kompilator meperingatkan bahwa ia
% tidak akan mengambil ulang unit karena sudah dalam rekompilasi kedua
unit_u_flag_for_reload=10051_U_Flag untuk mengambil ulang: $1
% Saat anda menggunakan flag \var{-vu}, kompilator meperingatkan bahwa ia
% harus mengambil ulang unit
unit_u_forced_reload=10052_U_Pemaksaan pengambilan ulang
% Saat anda menggunakan flag \var{-vu}, kompilator meperingatkan bahwa ia
% mengambil ulang unit karen unit diperlukan
unit_u_previous_state=10053_U_Keadaan $1 sebelumnya: $2
% Saat anda menggunakan flag \var{-vu}, kompilator menampilkan
% keadaan unit sebelumnya
unit_u_second_compile_unit=10054_U_Sudah mengompilasi $1, menyetel kompilasi kedua
% Saat anda menggunakan flag \var{-vu}, kompilator meperingatkan bahwa ia
% mulai mengompilasi ulang unit untuk kedua kali. Ini bisa terjadi dengan
% unit interdependensi.
unit_u_loading_unit=10055_U_Mengambil unit $1
% Saat anda menggunakan flag \var{-vu}, kompilator meperingatkan bahwa ia
% mulai mengambil unit.
unit_u_finished_loading_unit=10056_U_Selesai mengambil unit $1
% Saat anda menggunakan flag \var{-vu}, kompilator meperingatkan bahwa ia
% selesai mengambil unit.
unit_u_registering_new_unit=10057_U_Mendaftarkan unit baru $1
% Saat anda menggunakan flag \var{-vu}, kompilator meperingatkan bahwa ia
% telah menemukan unit baru dan mendaftarkannya dalam daftar internal.
unit_u_reresolving_unit=10058_U_Memecahkan ulang unit $1
% Saat anda menggunakan flag \var{-vu}, kompilator meperingatkan bahwa ia
% harus menghitung ulang data internal pada unit
unit_u_skipping_reresolving_unit=10059_U_Melewati pemecahan ulang unit $1, masih mengambil unit yang dipakai
% Saat anda menggunakan flag \var{-vu}, kompilator memperingatkan bahwa ia
% mulai melewati untuk menghitung ulang data internal pada unit karena di sana
% tidak ada data untuk dihitung ulang
% \end{description}
# EndOfTeX

#
#  Opsi
#
# 11041 is the last used one
#
unit_u_unload_resunit=10060_U_Unloading resource unit $1 (not needed)
% When you use the \var{-vu} flag, the compiler warns that it is unloading the
% resource handling unit, since no resources are used.
unit_e_different_wpo_file=10061_E_Unit $1 was compiled using a different whole program optimization feedback input ($2, $3); recompile it without wpo or use the same wpo feedback input file for this compilation invocation
% When a unit has been compiled using a particular whole program optimization (wpo) feedback file (\var{-FW<x>} \var{-OW<x>}),
% this compiled version of the unit is specialised for that particular compilation scenario and cannot be used in
% any other context. It has to be recompiled before you can use it in another program or with another wpo feedback input file.
unit_u_indirect_crc_changed=10062_U_Indirect interface (objects/classes) CRC changed for unit $1
% When you use the \var{-vu} flag, the compiler warns that the
% indirect CRC calculated for the unit (this is the CRC of all classes/objects/interfaces/$\ldots$
% in the interfaces of units directly or indirectly used by this unit in the interface) has been changed after the
% implementation has been parsed.
unit_u_ppu_invalid_memory_model=10063_U_PPU is compiled for another i8086 memory model
% This unit file was compiled for a different i8086 memory model and
% cannot be read.
% \end{description}
# EndOfTeX

#
#  Options
#
# 11057 is the last used one
#
option_usage=11000_O_$1 [opsi] <inputfile> [opsi]
# BeginOfTeX
%
% \section{Kesaslahan penanganan baris perintah}
% Seksi ini mendaftarkan kesalahan yang terjadi saat kompilator memproses
% baris perintah atau menangani file konfigurasi.
% \begin{description}
option_only_one_source_support=11001_W_Hanya satu file sumber didukung
% Anda dapat menetapkan hanya satu file sumber pada baris perintah. Yang
% pertama akan dikompilasi, yang lainnya akan diabaikan. Ini menunjukan bahwa
% anda melupakan tanda \var{'-'}.
option_def_only_for_os2=11002_W_File DEF hanya dapat dibuat untuk OS/2
% Opsi ini hanya ditetapkan ketika anda mengompilasi untuk OS/2
option_no_nested_response_file=11003_E_File respon berulang tidak didukung
% Anda tidak bisa mengulang file respon dengan opsi baris perintah \var{@file}.
option_no_source_found=11004_F_Nama file sumber tidak ada dalam baris perintah
% Kompilator mengharapkan nama file sumber pada baris perintah.
option_no_option_found=11005_N_Tidak ada opsi di dalam file konfig $1
% Kompilator tidak menemukan setiap opsi dalam file konfig.
option_illegal_para=11006_E_Parameter tidak benar: $1
% Anda menetapkan opsi yang tidak dikenal.
option_help_pages_para=11007_H_-? menulis halaman bantuan
% Ketika sebuah opsi tidak dikenal diberikan, pesan ini ditampilkan.
option_too_many_cfg_files=11008_F_Terlalu banyak file konfig yang diulang
% Anda hanya bisa mengulang sampai 16 file konfigurasi.
option_unable_open_file=11009_F_Tidak bisa membuka file $1
% File opsi tidak bisa ditemukan.
option_reading_further_from=11010_D_Membaca opsi lanjutan dari $1
% Ditampilkan saat anda menghidupkan catatan, dan kompilator beralih ke
% file opsi lain.
option_target_is_already_set=11011_W_Target sudah disetel ke: $1
% Ditampilkan jika lebih dari satu opsi \var{-T} ditetapkan.
option_no_shared_lib_under_dos=11012_W_Libs berbagi tidak didukung pada DOS platform, membalikan ke statis
% Jika anda menetapkan \var{-CD} untuk platform \dos, pesan ini ditampilkan.
% Kompilator hanya mendukung libari statis di bawah \dos
option_too_many_ifdef=11013_F_terlalu banyak IF(N)DEF
% Pernyataan \var{\#IF(N)DEF} dalam file opsi tidak seimbang dengan pernyataan
% \var{\#ENDIF}.
option_too_many_endif=11014_F_terlalu banyak ENDIF
% Pernyataan \var{\#IF(N)DEF} dalam file opsi tidak seimbang dengan pernyataan
% \var{\#ENDIF}.
option_too_less_endif=11015_F_Kondisional terbuka di akhir file
% Pernyataan \var{\#IF(N)DEF} dalam file opsi tidak seimbang dengan pernyataan
% \var{\#ENDIF}.
option_no_debug_support=11016_W_Pembuatan informasi debug tidak didukung oleh executable ini
% Dimungkinkan untuk memiliki executable kompilator yang tidak mendukung
% pembuatan info debug. Jika anda menggunakan executable seperti itu dengan
% saklar \var{-g}, peringatan ini akan ditampilkan.
option_no_debug_support_recompile_fpc=11017_H_Coba rekompilasi dengan -dGDB
% Dimungkinkan untuk memiliki executable kompilator yang tidak mendukung
% pembuatan info debug. Jika anda menggunakan executable seperti itu dengan
% saklar \var{-g}, peringatan ini akan ditampilkan.
option_obsolete_switch=11018_W_Anda menggunakan saklar tidak terpakai $1
% Ini memperingatkan anda saat anda menggunakan saklar yang tidak diperlukan/didukung lagi.
% Direkomendasikan bahwa anda menghapus saklar untuk menghindari masalah di masa
% mendatang, saat saklar mungkin diubah.
option_obsolete_switch_use_new=11019_W_Anda menggunakan saklar tidak terpakai $1, silahkan pakai $2
% Ini memperingatkan anda saat anda memakai saklar yang tidak didukung lagi.
% sekarang sebaliknya anda harus menggunakan saklar kedua.
% Direkomendasikan bahwa anda menghapus saklar untuk menghindari masalah di masa
% mendatang, saat saklar mungkin diubah.
option_switch_bin_to_src_assembler=11020_N_Beralih assembler ke penulisan sumber assembler standar
% Ini memberitahu anda bahwa assembler sudah diubah karena anda menggunakan
% saklar -a yang tidak bisa dipakai dengan penulis assembler biner.
option_incompatible_asm=11021_W_Output assembler yang dipilih "$1" tidak kompatibel dengan "$2"
option_asm_forced=11022_W_"$1" assembler dipaksa dipakai
% Output assembler yang dipilih tidak dapat menghasilkan
% file obyek dengan format yang benar. Oleh karena itu, sebaliknya
% assembler standar untuk target ini digunakan.
option_using_file=11026_T_Membaca opsi dari file $1
% Opsi juga dibaca dari file ini
option_using_env=11027_T_Membaca opsi dari lingkungan $1
% Opsi juga dibaca dari string lingkungan ini
option_handling_option=11028_D_Menangani opsi "$1"
% Info debug yang opsinya ditemukan akan ditangani
option_help_press_enter=11029__*** tekan enter ***
% Message shown when help is shown page per page. When pressing the ENTER
% Key, the next page of help is shown. If you press q and then ENTER, the
% compiler exits.
option_start_reading_configfile=11030_H_Mulai membaca file konfigurasi $1
% Memulai penguraian file konfigurasi.
option_end_reading_configfile=11031_H_Akhir pembacaan file konfigurasi $1
% Akhir penguraian file konfigurasi.
option_interpreting_option=11032_D_menginterpretasikan opsi "$1"
% The compiler is interpreting an option
option_interpreting_firstpass_option=11036_D_menginterpretasikan operan opsi pertama "$1"
% The compiler is interpreting an option for the first time.
option_interpreting_file_option=11033_D_menginterpretasikan opsi file "$1"
% The compiler is interpreting an option which it read from the configuration file.
option_read_config_file=11034_D_Membaca file konfigurasi "$1"
% The compiler is starting to read the configuration file.
option_found_file=11035_D_ditemukan nama file sumber "$1"
% Info tambahan mengenai opsi, ditampilkan saat anda menghidupkan opsi
% debug.
option_code_page_not_available=11039_E_Halaman kode tidak dikenal
% An unknown codepage for the source files was requested.
% The compiler is compiled with support for several codepages built-in.
% The requested codepage is not in that list. You will need to recompile
% the compiler with support for the codepage you need.
option_config_is_dir=11040_F_File konfigurasi $1 adalah sebuah direktori
% Direktori tidak bisa dipakai sebagai file konfigurasi.
option_confict_asm_debug=11041_W_Output assembler yang dipilih "$1" tidak bisa menghasilkan info debug, debugging dimatikan
% Output assembler yang dipilih tidak dapat membuat informasi debug,
% oleh karenanya opsi debug dimatikan.
%\end{description}
# EndOfTeX

#
# Logo (option -l)
#
option_ppc386_deprecated=11042_W_Use of ppc386.cfg is deprecated, please use fpc.cfg instead
% Using ppc386.cfg is still supported for historical reasons, however, for a multiplatform
% system the naming makes no sense anymore. Please continue to use fpc.cfg instead.
option_else_without_if=11043_F_In options file $1 at line $2 #ELSE directive without #IF(N)DEF found
% An \var{\#ELSE} statement was found in the options file without a matching \var{\#IF(N)DEF} statement.
option_unsupported_target=11044_F_Option "$1" is not, or not yet, supported on the current target platform
% Not all options are supported or implemented for all target platforms. This message informs you that a chosen
% option is incompatible with the currently selected target platform.
option_unsupported_target_for_feature=11045_F_The feature "$1" is not, or not yet, supported on the selected target platform
% Not all features are supported or implemented for all target platforms. This message informs you that a chosen
% feature is incompatible with the currently selected target platform.
option_dwarf_smart_linking=11046_N_DWARF debug information cannot be used with smart linking on this target, switching to static linking
% Smart linking is currently incompatible with DWARF debug information on most
% platforms, so smart linking is disabled in such cases.
option_ignored_target=11047_W_Option "$1" is ignored for the current target platform.
% Not all options are supported or implemented for all target platforms. This message informs you that a chosen
% option is ignored for the currently selected target platform.
option_debug_external_unsupported=11048_W_Disabling external debug information because it is unsupported for the selected target/debug format combination.
% Not all debug formats can be stored in an external file on all platforms. In particular, on
% Mac OS X only DWARF debug information can be stored externally.
option_dwarf_smartlink_creation=11049_N_DWARF debug information cannot be used with smart linking with external assembler, disabling static library creation.
% Smart linking is currently incompatble with DWARF debug information on most
% platforms, so smart linking is disabled in such cases.
option_invalid_macosx_deployment_target=11050_E_Invalid value for MACOSX_DEPLOYMENT_TARGET environment variable: $1
option_invalid_iphoneos_deployment_target=11051_E_Invalid value for IPHONEOS_DEPLOYMENT_TARGET environment variable: $1
% On Mac OS X, the MACOSX\_DEPLOYMENT\_TARGET/IPHONEOS\_DEPLOYMENT\_TARGET environment variable can be
% used to set the default target OS version. In case of Mac OS X, it has to be of the format
% XY.Z or XY.Z.AB with X, Y,Z , A and B all digits from 0-9.
% In case of iOS, it has to be X.Z.A, where X, Z and A can all be either 1 or 2
% digits from 0-9.
option_illegal_fpu_eabihf=11052_E_You must use a FPU type of VFPV2, VFPV3 or VFPV3_D16 when using the EABIHF ABI target
% The EABIHF (VFP hardfloat) ABI target can only be used with VFP FPUs.
option_w_unsupported_debug_format=11053_W_The selected debug format is not supported on the current target, not changing the current setting
% Not all targets support all debug formats (in particular, Stabs is not supported on 64 bit targets).
option_missing_arg=11054_E_argument to "$1" is missing
% Displayed when parameter must be followed by an argument.
option_malformed_para=11055_E_malformed parameter: $1
% Given argument is not valid for parameter.
option_smart_link_requires_external_linker=11056_W_Smart linking requires external linker
option_com_files_require_tiny_model=11057_E_Creating .COM files is not supported in the current memory model. Only the tiny memory model supports making .COM files.


%\end{description}
# EndOfTeX

#
#  Whole program optimization
#
# 12019 is the last used one
#
# BeginOfTeX
%
% \section{Whole program optimization messages}
% This section lists errors that occur when the compiler is performing
% whole program optimization.
% \begin{description}
wpo_cant_find_file=12000_F_Cannot open whole program optimization feedback file "$1"
% The compiler cannot open the specified feedback file with whole program optimization information.
wpo_begin_processing=12001_D_Processing whole program optimization information in wpo feedback file "$1"
% The compiler starts processing whole program optimization information found in the named file.
wpo_end_processing=12002_D_Finished processing the whole program optimization information in wpo feedback file "$1"
% The compiler has finished processing the whole program optimization information found in the named file.
wpo_expected_section=12003_E_Expected section header, but got "$2" at line $1 of wpo feedback file
% The compiler expected a section header in the whole program optimization file (starting with \%),
% but did not find it.
wpo_no_section_handler=12004_W_No handler registered for whole program optimization section "$2" at line $1 of wpo feedback file, ignoring
% The compiler has no handler to deal with the mentioned whole program optimization information
% section, and will therefore ignore it and skip to the next section.
wpo_found_section=12005_D_Found whole program optimization section "$1" with information about "$2"
% The compiler encountered a section with whole program optimization information, and according
% to its handler this section contains information usable for the mentioned purpose.
wpo_no_input_specified=12006_F_The selected whole program optimizations require a previously generated feedback file (use -Fw to specify)
% The compiler needs information gathered during a previous compilation run to perform the selected
% whole program optimizations. You can specify the location of the feedback file containing this
% information using the -Fw switch.
wpo_not_enough_info=12007_E_No collected information necessary to perform "$1" whole program optimization found
% While you pointed the compiler to a file containing whole program optimization feedback, it
% did not contain the information necessary to perform the selected optimizations. You most likely
% have to recompile the program using the appropriate -OWxxx switch.
wpo_no_output_specified=12008_F_Specify a whole program optimization feedback file to store the generated info in (using -FW)
% You have to specify the feedback file in which the compiler has to store the whole program optimization
% feedback that is generated during the compilation run. This can be done using the -FW switch.
wpo_output_without_info_gen=12009_E_Not generating any whole program optimization information, yet a feedback file was specified (using -FW)
% The compiler was instructed to store whole program optimization feedback into a file specified using -FW,
% but not to actually generated any whole program optimization feedback. The classes of to be
% generated information can be specified using -OWxxx.
wpo_input_without_info_use=12010_E_Not performing any whole program optimizations, yet an input feedback file was specified (using -Fw)
% The compiler was not instructed to perform any whole program optimizations (no -Owxxx parameters),
% but nevertheless an input file with such feedback was specified (using -Fwyyy). Since this can
% indicate that you forgot to specify an -Owxxx parameter, the compiler generates an error in this case.
wpo_skipping_unnecessary_section=12011_D_Skipping whole program optimization section "$1", because not needed by the requested optimizations
% The whole program optimization feedback file contains a section with information that is not
% required by the selected whole program optimizations.
wpo_duplicate_wpotype=12012_W_Overriding previously read information for "$1" from feedback input file using information in section "$2"
% The feedback file contains multiple sections that provide the same class of information (e.g.,
% information about which virtual methods can be devirtualized). In this case, the information in last encountered
% section is used. Turn on debugging output (-vd) to see which class of information is provided by each section.
wpo_cannot_extract_live_symbol_info_strip=12013_E_Cannot extract symbol liveness information from program when stripping symbols, use -Xs-
% Certain symbol liveness collectors extract the symbol information from the linked program. If the symbol information
% is stripped (option -Xs), this is not possible.
wpo_cannot_extract_live_symbol_info_no_link=12014_E_Cannot extract symbol liveness information from program when when not linking
% Certain symbol liveness collectors extract the symbol information from the linked program. If the program is not
% linked by the compiler, this is not possible.
wpo_cannot_find_symbol_progs=12015_F_Cannot find "$1" or "$2" to extract symbol liveness information from linked program
% Certain symbol liveness collectors need a helper program to extract the symbol information from the linked program.
% This helper program is normally 'nm', which is part of the GNU binutils.
wpo_error_reading_symbol_file=12016_E_Error during reading symbol liveness information produced by "$1"
% An error occurred during the reading of the symbol liveness file that was generated using the 'nm' or 'objdump' program. The reason
% can be that it was shorter than expected, or that its format was not understood.
wpo_error_executing_symbol_prog=12017_F_Error executing "$1" (exitcode: $2) to extract symbol information from linked program
% Certain symbol liveness collectors need a helper program to extract the symbol information from the linked program.
% The helper program produced the reported error code when it was run on the linked program.
wpo_symbol_live_info_needs_smart_linking=12018_E_Collection of symbol liveness information can only help when using smart linking, use -CX -XX
% Whether or not a symbol is live is determined by looking whether it exists in the final linked program.
% Without smart linking/dead code stripping, all symbols are always included, regardless of whether they are
% actually used or not. So in that case all symbols will be seen as live, which makes this optimization ineffective.
wpo_cant_create_feedback_file=12019_E_Cannot create specified whole program optimisation feedback file "$1"
% The compiler is unable to create the file specified using the -FW parameter to store the whole program optimisation information.
%\end{description}
# EndOfTeX


#
# Logo (option -l)
#
option_logo=11023_[
Free Pascal Compiler versi $FPCFULLVERSION [$FPCDATE] untuk $FPCCPU
Hak Cipta (c) 1993-2020 oleh Florian Klaempfl and others
]

#
# Info (option -i)
#
option_info=11024_[
Free Pascal Compiler versi $FPCVERSION

Tanggal Kompilator   : $FPCDATE
Target CPU Kompilator: $FPCCPU

Target didukung:
  $OSTARGETS

Set instruksi CPU didukung:
  $INSTRUCTIONSETS

Set instruksi FPU didukung:
  $FPUINSTRUCTIONSETS

Optimasi Didukung:
  $OPTIMIZATIONS

Program ini datang bersama GNU General Public Licence
Untuk informasi lebih jauh baca COPYING.v2

Laporkan bugs, saran, dll ke:
                 http://bugs.freepascal.org
]

#
# Halaman bantuan (opsi -? dan -h)
#
# Karakter pertama pada baris menunjukan siapa yang akan menampilkan baris
# ini, kemungkinan saat ini ialah :
#    * = setiap target
#    3 = 80x86 target
#    6 = 680x0 target
#    e = hanya dalam mode debug diperluas
#    P = PowerPC target
#    S = Sparc target
#    V = Target mesin virtual
# Karakter kedua juga menunjukan siapa yang akan menampilkan baris ini,
# (jika karakter di atas adalah BENAR) kemungkinan saat ini ialah :
#    * = setiap orang
#    g = dengan info GDB yang didukung oleh kompilator
#    O = OS/2
#    L = Sistem UNIX
#  Karakter ketiga mewakili tingkat indentasi.
#
option_help_pages=11025_[
**0*_Tambah + setelah opsi saklar boolean untuk menghidupkannya, - untuk mematikan
**1a_Kompilator tidak menghapus file assembler yang dibuat
**2al_Daftarkan baris kode sumber dalam file assembler
**2an_Daftarkan info node dalam file assembler
*L2ap_Gunakan pipa daripada pembuatan file assembler sementara
**2ar_Daftarkan info alokasi/pelepasan register dalam file assembler
**2at_Daftarkan info alokasi/pelepasan temp dalam file assembler
**1A<x>_Format output:
**2Adefault_Gunakan assembler standar
3*2Aas_Assemble menggunakan GNU AS
3*2Anasmcoff_File COFF (Go32v2) menggunakan Nasm
3*2Anasmelf_File ELF32 (Linux) menggunakan Nasm
3*2Anasmwin32_File obyek Win32 menggunakan Nasm
3*2Anasmwdosx_File obyek Win32/WDOSX menggunakan Nasm
3*2Awasm_File Obj menggunakan Wasm (Watcom)
3*2Anasmobj_File Obj menggunakan Nasm
3*2Amasm_File Obj menggunakan Masm (Microsoft)
3*2Atasm_File Obj menggunakan Tasm (Borland)
3*2Aelf_ELF (Linux) menggunakan penulis internal
3*2Acoff_COFF (Go32v2) menggunakan penulis internal
3*2Apecoff_PE-COFF (Win32) menggunakan penulis internal
4*2Aas_Rangkai menggunakan GNU AS
6*2Aas_File-0 Unix menggunakan GNU AS
6*2Agas_GNU Motorola assembler
6*2Amit_Sintaks MIT (GAS lama)
6*2Amot_Standard Motorola assembler
A*2Aas_Rangkai menggunakan GNU AS
P*2Aas_Rangkai menggunakan GNU AS
S*2Aas_Rangkai menggunakan GNU AS
**1b_Hasilkan info browser
**2bl_Hasilkan info simbol lokal
**1B_Bangun seluruh modul
**1C<x>_Opsi pembuatan kode:
**2Cc<x>_Setel konvensi pemanggilan standar ke <x>
**2CD_Buat juga librari dinamis (tidak didukung)
**2Ce_Kompilasi dengan opcode pecahan diemulasikan
**2Cf<x>_Pilih set instruksi fpu yang dipakai, lihat fpc -i untuk nilai yang mungkin
**2CF<x>_Minimal presisi konstan pecahan (standar, 32, 64)
**2Cg_Hasilkan kode PIC
**2Ch<n>_<n> byte heap (antara 1023 dan 67107840)
**2Ci_Pemeriksaan-IO
**2Cn_Abaikan tahap linking
**2Co_Periksa kelebihan untuk operasi integer
**2Cp<x>_Pilih set instruksi, lihat fpc -i untuk nilai yang mungkin
**2CP<x>=<y>_ setelan pemadatan
**3CPPACKSET=<y>_ <y> setel alokasi: 0, 1 atau DEFAULT atau NORMAL, 2, 4 dan 8
**2Cr_Pemeriksaan jangkauan
**2CR_Verifikasi kebenaran pemanggilan metode obyek
**2Cs<n>_Setel besar stack menjadi <n>
**2Ct_Pemeriksaan stack
**2CX_Buat juga librari smartlinked
**1d<x>_Definisikan simbol <x>
**1D_Hasilkan file DEF
**2Dd<x>_Setel deskripsi menjadi <x>
**2Dv<x>_Setel versi DLL menjadi <x>
*O2Dw_Aplikasi PM
**1e<x>_Setel path ke executable
**1E_Sama seperti -Cn
**1fPIC_Sama seperti -Cg
**1F<x>_Setel nama file dan path:
**2Fa<x>[,y]_(untuk sebuah program) ambil unit <x> dan [y] sebelum uses diuraikan
**2Fc<x>_Setel input codepage menjadi <x>
**2FC<x>_Setel nama biner kompilator RC menjadi <x>
**2FD<x>_Setel direktori ke mana untuk menjcari utilitas kompilator
**2Fe<x>_Alihkan output kesalahan ke <x>
**2Ff<x>_Tambah <x> ke path kerangka kerja (hanya Darwin)
**2FE<x>_Setel path output exe/unit ke <x>
**2Fi<x>_Tambah <x> ke path include
**2Fl<x>_Tambah <x> ke path librari
**2FL<x>_Gunakan <x> sebagai linker dinamis
**2Fm<x>_Ambil tabel konversi unicode dari <x>.txt dalam dir kompilator
**2Fo<x>_Tambah <x> ke path obyek
**2Fr<x>_Ambil file pesan kesalahan <x>
**2FR<x>_Setel linker resource (.res) ke <x>
**2Fu<x>_Tambah <x> ke path unit
**2FU<x>_Setel path output unit ke <x>, ganti -FE
*g1g_Hasilkan informasi debug (format standar untuk target)
*g2gc_Hasilkan pemeriksaan untuk pointer
*g2gh_Gunakan unit heaptrace (untuk men-debug kebocoran/kerusakan memori)
*g2gl_Gunakan info baris unit (tampilkan info lebih lengkap dengan backtraces)
*g2go<x>_Setel opsi informasi debug
*g3godwarfsets_ Hidupkan informasi setelan debug Dwarf (pecahkan gdb < 6.5)
*g2gp_Siapkan jenis huruf dalam nama simbol stabs
*g2gs_Hasilkan informasi debug stabs
*g2gt_Bersihkan variabel lokal (untuk mendeteksi uses tak terinisialisasi)
*g2gv_Hasilkan program yang bisa dilacak oleh valgrind
*g2gw_Hasilkan informasi debug dwarf-2 (sama seperti -gw2)
*g2gw2_Hasilkan informasi debug dwarf-2
*g2gw3_Hasilkan informasi debug dwarf-3
**1i_Informasi
**2iD_Kembalikan tanggal kompilator
**2iV_Kembalikan versi pendek kompilator
**2iW_Kembalikan versi lengkap kompilator
**2iSO_Kemblikan OS kompilator
**2iSP_Kembalikan prosesor host kompilator
**2iTO_Kembalikan OS target
**2iTP_Kembalikan prosesor target
**1I<x>_Tambah <x> ke path include
**1k<x>_Operkan <x> ke linker
**1l_Tulis logo
**1M<x>_Setel mode bahasa menjadi <x>
**2Mfpc_Dialek Free Pascal (standar)
**2Mobjfpc_Mode FPC dengan dukungan Obyek Pascal
**2Mdelphi_Mode kompatibilitas Delphi 7
**2Mtp_Mode kompatibilitas TP/BP 7.0
**2Mmacpas_Mode kompatibilitas dialek Macintosh Pascal
**1n_Jangan baca file konfigurasi standar
**1o<x>_Ubah nama executable yang dihasilkan ke <x>
**1O<x>_Optimasi:
**2O-_Matikan optimasi
**2O1_Optimasi tingkat 1 (cepat dan ramah debugger)
**2O2_Optimasi tingkat 2 (-O1 + optimasi cepat)
**2O3_Optimasi tingkat 3 (-O2 + optimasi lambat)
**2Oa<x>=<y>_Setel penjajaran
**2Oo[NO]<x>_Hidupkan atau matikan optimasi, lihat fpc -i untuk nilai yang mungkin
**2Op<x>_Setel cpu target untuk optimasi, lihat fpc -i untuk nilai yang mungkin
**2Os_Optimasi untuk ukuran daripada kecepatan
**1pg_Hasilkan kode profil untuk gprof (definisikan FPC_PROFILE)
**1R<x>_Gaya pembacaan assembler:
**2Rdefault_Gunakan assembler standar untuk target
3*2Ratt_Baca gaya assembler AT&T
3*2Rintel_Baca gaya assembler Intel
4*2Ratt_Baca gaya assembler AT&T
4*2Rintel_Baca gaya assembler Intel
8*2Ratt_Baca gaya assembler AT&T
8*2Rintel_Baca gaya assembler Intel
6*2RMOT_Baca gaya assembler Motorola
**1S<x>_Opsi sintaks:
**2S2_Sama seperti -Mobjfpc
**2Sc_Dukung operator seperti C (*=,+=,/= and -=)
**2Sa_Hidupkan assertions
**2Sd_Sama seperti -Mdelphi
**2Se<x>_Opsi kesalahan. <x> adalah kombinasi dari yang berikut:
**3*_<n> : Kompilator batal setelah <n> kesalahan (standarnya 1)
**3*_w : Kompilator juga batal setelah peringatan
**3*_n : Kompilator juga batal setelah catatan
**3*_h : Kompilator juga batal setelah petunjuk
**2Sg_Hidupkan LABEL dan GOTO (standar dalam -Mtp dan -Mdelphi)
**2Sh_Gunakan ansistrings secara standar daripada shortstrings
**2Si_Hidupkan inlining procedures/functions yang dideklarisak sebagai "inline"
**2Sk_Ambil unit fpcylix
**2SI<x>_Setel gaya antarmuka ke <x>
**3SIcom_Antarmuka kompatibel COM (standar)
**3SIcorba_Antarmuka kompatibel CORBA
**2Sm_Dukung makro seperti C (global)
**2So_Sama seperti -Mtp
**2Ss_Nama konstruktor harus init (destruktor harus done)
**2Sx_Hidupkan kata kunci exception (standar dalam mode Delphi/ObjFPC)
**1s_Jangan panggil assembler dan linker
**2sh_Hasilkan naskah untuk me-link pada host
**2st_Hasilkan naskah untuk me-link pada target
**2sr_Lewati tahap alokasi register (gunakan dengan -alr)
**1T<x>_Sistem operasi target:
3*2Temx_OS/2 via EMX (termasuk EMX/RSX extender)
3*2Tfreebsd_FreeBSD
3*2Tgo32v2_Versi 2 pada DJ Delorie DOS extender
3*2Tlinux_Linux
3*2Tnetbsd_NetBSD
3*2Tnetware_Novell Netware Module (clib)
3*2Tnetwlibc_Novell Netware Module (libc)
3*2Topenbsd_OpenBSD
3*2Tos2_OS/2 / eComStation
3*2Twatcom_Watcom kompatibel DOS extender
3*2Twdosx_WDOSX DOS extender
3*2Twin32_Windows 32 Bit
3*2Twince_Windows CE
4*2Tlinux_Linux
6*2Tamiga_Commodore Amiga
6*2Tatari_Atari ST/STe/TT
6*2Tlinux_Linux/m68k
6*2Tmacos_Macintosh m68k (tidak didukung)
6*2Tpalmos_PalmOS
A*2Tlinux_Linux
A*2Twince_Windows CE
P*2Tamiga_AmigaOS pada PowerPC
P*2Tdarwin_Darwin dan Mac OS X pada PowerPC
P*2Tlinux_Linux pada PowerPC
P*2Tmacos_Mac OS (klasik) pada PowerPC
P*2Tmorphos_MorphOS
S*2Tlinux_Linux
**1u<x>_Simbol tidak didefinisikan <x>
**1U_Opsi unit:
**2Un_Jangan periksa di mana nama unit sama dengan nama file
**2Ur_Hasilkan file unit rilis (jangan dikompilasi secara otomatis)
**2Us_Kompilasi unit sistem
**1v<x>_Ditampilkan. <x> adalah kombinasi dari huruf berikut:
**2*_e : Tampilkan kesalahan (default) 0 : Jangan tampilkan (kecuali kesalahan)
**2*_w : Tampilkan peringatan          u : Tampilkan info unit
**2*_n : Tampilkan catatan             t : Tampilkan file yang dicoba/dipakai
**2*_h : Tampilkan petunjuk            c : Tampilkan kondisional
**2*_i : Tampilkan info umum           d : Tampilkan info debug
**2*_l : Tampilkan nomor baris         r : Mode kompatibilitas Rhide/GCC
**2*_a : Tampilkan semuanya            x : Info executable (hanya Win32)
**2*_b : Tulis pesan nama file dengan path lengkap
**2*_v : Tulis fpcdebug.txt dengan     p : Tulis tree.log dengan susunan urai
**2*_    banyak info debug
3*1W<x>_Opsi spesifik-target (target)
A*1W<x>_Opsi spesifik-target (target)
P*1W<x>_Opsi spesifik-target (target)
3*2Wb_Buat sebuah bundel daripada sebuah librari (Darwin)
P*2Wb_Buat sebuah bundel daripada sebuah librari (Darwin)
p*2Wb_Buat sebuah bundel daripada sebuah librari (Darwin)
3*2WB_Buat citra relokatabel (Windows)
A*2WB_Buat citra relokatabel (Windows, Symbian)
3*2WC_Tetapkan aplikasi tipe konsol (EMX, OS/2, Windows)
A*2WC_Tetapkan aplikasi tipe konsol (Windows)
P*2WC_Tetapkan aplikasi tipe konsol (Mac OS Klasik)
3*2WD_Gunakan DEFFILE untuk mengekspor fungsi DLL atau EXE (Windows)
A*2WD_Gunakan DEFFILE untuk mengekspor fungsi DLL atau EXE (Windows)
3*2WF_Tetapkan aplikasi tipe layar-penuh (EMX, OS/2)
3*2WG_Tetapkan aplikasi tipe grafis (EMX, OS/2, Windows)
A*2WG_Tetapkan aplikasi tipe grafis (Windows)
P*2WG_Tetapkan aplikasi tipe grafis (Classic Mac OS)
3*2WN_Jangan hasilkan kode relokasi, diperlukan untuk debugging (Windows)
A*2WN_Jangan hasilkan kode relokasi, diperlukan untuk debugging (Windows)
3*2WR_Hasilkan kode relokasi (Windows)
A*2WR_Hasilkan kode relokasi (Windows)
P*2WT_Tetapkan aplikasi tipe piranti MPW (Mac OS Klasik)
**1X_Opsi executable:
**2Xc_Oper --shared/-dynamic ke linker (BeOS, Darwin, FreeBSD, Linux)
**2Xd_Jangan gunakan path pencarian librari standar (diperlukan untuk kompilasi silang)
**2Xe_Gunakan linker eksternal
**2XD_Coba untuk me-link unit secara dinamis  (definisikan FPC_LINK_DYNAMIC)
**2Xi_Gunakan linker internal
**2Xm_Hasilkan peta link
**2XM<x>_Set nama rutin program 'main' (standarnya 'main')
**2XP<x>_Awali nama binutils dengan prefiks <x>
**2Xr<x>_Setel path pencarian librari ke <x> (diperlukan untuk kompilasi silang)
**2Xs_Buang semua simbol dari executable
**2XS_Coba untuk me-link unit secara statis (standar, definisikan FPC_LINK_STATIC)
**2Xt_Link dengan librari statis (-static dioper ke linker)
**2XX_Coba untuk smartlink unit             (definisikan FPC_LINK_SMART)
**1*_
**1?_Tampilkan bantuan ini
**1h_Tampilkan bantuan ini tanpa menunggu
]

#
# The End...
%%% parser_e_only_class_methods_via_class_ref=03053_E_Hanya metode kelas dapat dirujuk dengan referensi kelas
% Kesalahan ini terjadi dalam situasi seperti berikut:
% \begin{verbatim}
% Type :
%    Tclass = Class of Tobject;
%
% Var C : TClass;
%
% begin
% ...
% C.free
% \end{verbatim}
% \var{Free} bukan metode kelas dan tidak bisa dipanggil dengan referensi
% kelas.
%%% parser_e_only_class_methods=03054_E_Hanya metode kelas bisa diakses dalam metode kelas
% Ini terkait dengan kesalahan sebelumnya. Anda tidak bisa memanggil metode obyek dari
% dalam metode kelas. Kode berikut akan menghasilkan kesalahan ini:
% \begin{verbatim}
% class procedure tobject.x;
%
% begin
%   free
% \end{verbatim}
% Karena free adalah metode normal sebuah kelas, ia tidak bisa dipanggil dari
% metode kelas.
%%% parser_e_absolute_only_one_var=03095_E_absolute hanya bisa dikaitkan ke satu variabel
% Anda tidak bisa menetapkan lebih dari satu variabel sebelum direktif \var{absolute}.
% Selanjutnya, konstruksi berikut akan menghasilkan kesalahan ini:
% \begin{verbatim}
% Var Z : Longint;
%     X,Y : Longint absolute Z;
% \end{verbatim}
% \item [ absolute hanya bisa dikaitkan ke sebuah var atau const ]
% Alamat dari direktif \var{absolute} hanya dapat mengarah ke variabel atau
% tipe konstan. Oleh karena itu, kode berikut akan menghasilkan kesalahan ini:
% \begin{verbatim}
%   Procedure X;
%
%  var p : longint absolute x;
% \end{verbatim}
%
%%% parser_e_type_and_var_only_in_generics=03224_E_VAR dan TYPE hanya dibolehkan dalam generik
% Penggunaan VAR dan TYPE untuk mendeklarasikan tipe baru di dalam sebuah obyek
% hanya dibolehkan di dalam generik.
%%% parser_f_no_anonymous_specializations=03232_F_Spesialisasi generik anonim tidak diperbolehkan
% Sesuatu seperti
% \begin{verbatim}
% var
%   MyLinkedList: specialize TLinkedList<TFixedString15>;
% \end{verbatim}
% tidak dibolehkan. Deklarasikan tipe spesialisasi lebih dulu:
% \begin{verbatim}
% type
%   TMyLinkedList = specialize TLinkedList<TFixedString15>;
% var
%   MyLinkedList: TMyLinkedList
% \end{verbatim}
%%% type_w_mixed_signed_unsigned=04035_W_Mencampur ekspresi bertanda dan longword memberikan hasil 64bit
% Jika anda membagi (atau menghitung modulus dari) ekspresi bertanda dengan 
% longword (atau sebaliknya), atau jika anda memiliki overflow dan/atau 
% menghidupkan pemeriksaan jangkauan dan menggunakan ekspresi aritmatika
% (+, -, *, div, mod) dalam kedua angka bertanda dan longwords muncul,
% maka kesemuanya harus dievaluasi dalam 64bit yang lebih lambat daripada 
% aritmatika 32bit normal. Anda bisa menghindari ini dengan typecasting satu 
% operand agar ia sesuai tipe hasil atas yang lainnya.
%%% type_w_signed_unsigned_always_false=04044_W_Perbandingan selalu salah karena jangkauan nilai
% Ada perbandingan antara nilai unsigned dan konstan bertanda yang kurang dari
% nol. Karena promosi tipe, pernyataan akan selalu dievaluasi menjadi false.
% Typecast secara eksplisit konstan ke jangkauan yang benar guna menghindari masalah ini.
%%% type_w_signed_unsigned_always_true=04045_W_Perbandingan selalu true karena jangkauan nilai
% Ada perbandingan antara nilai unsigned dan konstan bertanda yang kurang dari
% nol. Karena promosi tipe, pernyataan akan selalu dievaluasi menjadi true.
% Typecast secara eksplisit konstan ke jangkauan yang benar guna menghindari masalah ini.
