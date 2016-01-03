Type
  TDosKeyboard = Class(TObject)
  Private
    { internal key functions }
    Procedure insert(_key : TPTCKey);
    Function remove : TPTCKey;
    Function ready : Boolean;

    { data }
    m_key : Boolean;

    { modifiers }
    m_alt : Boolean;
    m_shift : Boolean;
    m_control : Boolean;

    { key buffer }
    m_head : Integer;
    m_tail : Integer;
    m_buffer : Array[0..1023] Of TPTCKey;
  Public
    { setup }
    Constructor Create;
    Destructor Destroy; Override;

    { input }
    Procedure internal_ReadKey(k : TPTCKey);
    Function internal_PeekKey(k : TPTCKey) : Boolean;
  End;
