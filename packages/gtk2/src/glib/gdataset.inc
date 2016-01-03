// included by glib2.pas

{$IFDEF read_forward_definitions}
  PPGData   = ^PGData;
  PGData    =  pointer;
{$ENDIF read_forward_definitions}

//------------------------------------------------------------------------------

{$IFDEF read_interface_types}
  TGDataForeachFunc = procedure (key_id:TGQuark; data:gpointer; user_data:gpointer);cdecl;
<<<<<<< HEAD
<<<<<<< HEAD
  GDestroyNotify = procedure (data:gpointer);cdecl;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
{$ENDIF read_interface_types}

{$IFDEF read_interface_rest}
{ Keyed Data List }

procedure g_datalist_init(datalist:PPGData);cdecl;external gliblib name 'g_datalist_init';
procedure g_datalist_clear(datalist:PPGData);cdecl;external gliblib name 'g_datalist_clear';
function  g_datalist_id_get_data(datalist:PPGData; key_id:TGQuark):gpointer;cdecl;external gliblib name 'g_datalist_id_get_data';
procedure g_datalist_id_set_data_full(datalist:PPGData; key_id:TGQuark; data:gpointer; destroy_func:TGDestroyNotify);cdecl;external gliblib name 'g_datalist_id_set_data_full';
function  g_datalist_id_remove_no_notify(datalist:PPGData; key_id:TGQuark):gpointer;cdecl;external gliblib name 'g_datalist_id_remove_no_notify';
procedure g_datalist_foreach(datalist:PPGData; func:TGDataForeachFunc; user_data:gpointer);cdecl;external gliblib name 'g_datalist_foreach';

{macros}
procedure g_datalist_id_set_data (datalist: PPGData; key_id:TGQuark; data: gpointer);
procedure g_datalist_id_remove_data (datalist: PPGData; key_id:TGQuark);
function  g_datalist_get_data(datalist: PPGData; key_str:PGChar):PPGData;
procedure g_datalist_set_data_full(datalist: PPGData; key_str:PGChar; data:gpointer; destroy_func:TGDestroyNotify);
procedure g_datalist_set_data (datalist: PPGData; key_str:PGChar; data:gpointer);
procedure g_datalist_remove_no_notify(datalist: PPGData; key_str:PGChar);
procedure g_datalist_remove_data(datalist:PPGData; key_str:PGChar);


{ Location Associated Keyed Data  }

function  g_dataset_id_get_data(dataset_location:gconstpointer; key_id:TGQuark):gpointer;cdecl;external gliblib name 'g_dataset_id_get_data';
procedure g_dataset_id_set_data_full(dataset_location:gconstpointer; key_id:TGQuark; data:gpointer; destroy_func:TGDestroyNotify);cdecl;external gliblib name 'g_dataset_id_set_data_full';
function  g_dataset_id_remove_no_notify(dataset_location:gconstpointer; key_id:TGQuark):gpointer;cdecl;external gliblib name 'g_dataset_id_remove_no_notify';
procedure g_dataset_foreach(dataset_location:gconstpointer; func:TGDataForeachFunc; user_data:gpointer);cdecl;external gliblib name 'g_dataset_foreach';

{macros}
procedure g_dataset_id_set_data(location: gconstpointer; key_id:TGQuark; data:gpointer);
procedure g_dataset_id_remove_data(location: gconstpointer; key_id:TGQuark);
function  g_dataset_get_data(location: gconstpointer; key_str:PGChar): gpointer;
procedure g_dataset_set_data_full(location: gconstpointer; key_str:PGChar; data:gpointer; destroy_func:TGDestroyNotify);
procedure g_dataset_remove_no_notify(location: gconstpointer; key_str:PGChar);
procedure g_dataset_set_data(location: gconstpointer; key_str:PGChar; data:gpointer);
procedure g_dataset_remove_data(location:gconstpointer; key_str: PGChar);
{$ENDIF read_interface_rest}

// included by glib2.pas


