<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
Ext.require(['Ext.data.*', 'Ext.grid.*']);

Ext.define('Person', {
    extend: 'Ext.data.Model',
    fields: [{
        name: 'ID',
        type: 'int',
        useNull: true
    }, 'LOGIN', 'NAME', 'EMAIL'],
    validations: [{
        type: 'length',
        field: 'LOGIN',
        min: 1
    }, {
        type: 'length',
        field: 'NAME',
        min: 1
    }, {
        type: 'length',
        field: 'EMAIL',
        min: 1
    }]
});

Ext.onReady(function(){

    var store = Ext.create('Ext.data.Store', {
        autoLoad: true,
        autoSync: true,
        model: 'Person',
        proxy: {
            type: 'ajax',

			api: {
				  read: "/cgi-bin/extgrid.exe/Provider/Read/",
				  update: "/cgi-bin/extgrid.exe/Provider/Update/",
				  create: "/cgi-bin/extgrid.exe/Provider/Insert/",
				  destroy: "/cgi-bin/extgrid.exe/Provider/Delete/"
			},

            reader: {
                type: 'json',
                successProperty: 'success',
                root: 'rows',
                messageProperty: 'message'
            },
            writer: {
                type: 'json',
				encode: true,
                //writeAllFields: false,
                root: 'rows'
            }
        },
        listeners: {
            write: function(store, operation){
                var record = operation.getRecords()[0],
                    name = Ext.String.capitalize(operation.action),
                    verb;
                    
                if (name == 'Destroy') {
                    record = operation.records[0];
                    verb = 'delete';
                } else if (name == 'Update') {
                    verb = 'update';
                } else if (name == 'Create') {
                    verb = 'create';
                }
				Ext.example.msg(verb, Ext.String.format("{0} user: {1}", verb, record.get('NAME')));
            }
        }
    });
    
    var rowEditing = Ext.create('Ext.grid.plugin.RowEditing');
    
    var grid = Ext.create('Ext.grid.Panel', {
        renderTo: document.body,
        plugins: [rowEditing],
        width: 400,
        height: 300,
        frame: true,
        title: 'Users',
        store: store,
        iconCls: 'icon-user',
        columns: [
			{
            text: 'LOGIN',
            flex: 1,
            sortable: true,
            dataIndex: 'LOGIN',
            field: {
                xtype: 'textfield'
            }
        }, {
            header: 'NAME',
            width: 80,
            sortable: true,
            dataIndex: 'NAME',
            field: {
                xtype: 'textfield'
            }
        }, {
            text: 'EMAIL',
            width: 80,
            sortable: true,
            dataIndex: 'EMAIL',
            field: {
                xtype: 'textfield'
            }
        }, {
            text: 'LASTLOGIN',
            width: 80,
            sortable: true,
            dataIndex: 'LASTLOGIN',
            field: {
                xtype: 'textfield'
            }
        }],
        dockedItems: [{
            xtype: 'toolbar',
            items: [{
                text: 'Add',
                iconCls: 'icon-add',
                handler: function(){
                    // empty record
                    store.insert(0, new Person());
                    rowEditing.startEdit(0, 0);
                }
            }, '-', {
                itemId: 'delete',
                text: 'Delete',
                iconCls: 'icon-delete',
                disabled: true,
                handler: function(){
                    var selection = grid.getView().getSelectionModel().getSelection()[0];
                    if (selection) {
                        store.remove(selection);
                    }
                }
            }]
        }]
    });
    grid.getSelectionModel().on('selectionchange', function(selModel, selections){
        grid.down('#delete').setDisabled(selections.length === 0);
    });
});
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
Ext.ns('FPC');

FPC.ShowPage = function () {
  var myproxy = new Ext.data.HttpProxy ( {
    api : {
      read: "extgrid.cgi/Provider/Read/name/",
      update: "extgrid.cgi/Provider/Update/name/",
      create: "extgrid.cgi/Provider/Insert/name/",
      destroy: "extgrid.cgi/Provider/Delete/name/"
    }
  });
  var myreader = new Ext.data.JsonReader ({
      root: "rows",
      idProperty: "ID",
      messageProperty: 'message', // Must be specified here
      fields: ["ID","LOGIN","NAME","EMAIL", "LASTLOGIN"]
  });
  var mywriter = new Ext.data.JsonWriter({
      encode: true,
      writeAllFields: true,
      idProperty: "ID"
  });
  var data = new Ext.data.Store({
    proxy: myproxy,
    reader: myreader,
    writer: mywriter,
    autoSave: false,
    idProperty: "ID",
  });
  // Listen to errors.
  data.addListener('exception', function(proxy, type, action, options, res) {
    if (type === 'remote') {
        Ext.Msg.show({
            title: 'REMOTE EXCEPTION',
            msg: res.message, 
            icon: Ext.MessageBox.ERROR,
            buttons: Ext.Msg.OK
        });
    }
  });
  data.load({ params:{start: 0, limit: 30}});
  var grid = new Ext.grid.EditorGridPanel({
    renderTo: Ext.getBody(),
    frame: true,
    title: "Known users",
    height: 600,
    width: 800,
    store: data,
    columns: [
      {header: 'ID', dataIndex: "ID", sortable: true, hidden: true},
      {header: 'Login', dataIndex: "LOGIN", sortable: true, editor: new Ext.form.TextField({allowBlank: false})},
      {header: 'Name', dataIndex: "NAME", sortable: true, editor: new Ext.form.TextField({allowBlank: false})},
      {header: 'Email', dataIndex: "EMAIL", sortable: true, editor: new Ext.form.TextField({allowBlank: false})},
      {header: 'Last login', dataIndex: "LASTLOGIN", sortable: true, editor: new Ext.form.TextField()},
    ],
    bbar: new Ext.PagingToolbar({
      pageSize: 30,
      store: data,
      displayInfo: true
    }),
    tbar : [{
            text: 'Add',
            iconCls: 'icon-add',
            handler: function(btn, ev) {
              var u = new grid.store.recordType();
              grid.stopEditing();
              grid.store.insert(0, u);
              grid.startEditing(0, 1);
            },
            scope: grid
        }, '-', {
            text: 'Delete',
            iconCls: 'icon-delete',
            handler: function(btn, ev) {
	      var index = grid.getSelectionModel().getSelectedCell();
	      if (!index) {
		  return false;
	      }
	      var rec = grid.store.getAt(index[0]);
	      grid.store.remove(rec);
	      },
            scope: grid
        }, '-', {
            text: 'Save',
            iconCls: 'icon-save',
            handler: function(btn, ev) {
	      grid.store.save();
	    },
            scope: grid
        }
	]
  });
  grid.show();
}
Ext.onReady(FPC.ShowPage);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
