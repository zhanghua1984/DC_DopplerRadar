// RadarDlg.h : header file
//

#if !defined(AFX_COMMTESTDLG_H__094EF006_CBE0_11D7_B4F9_00E04C74763F__INCLUDED_)
#define AFX_COMMTESTDLG_H__094EF006_CBE0_11D7_B4F9_00E04C74763F__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

/////////////////////////////////////////////////////////////////////////////
// CRadarDlg dialog

class CRadarDlg : public CDialog
{
// Construction
public:
	int Counters();
	long m_rxlen;
	long m_txlen;
	long m_crclen;
	long m_counter;
	long m_lostframe;
	long m_countertodispatch;
	BOOL b_fileopen;
	void DispStatus();
	void CloseConnection();
	BOOL ProcessCOMMNotification (UINT wParam,long lParam);
	BOOL m_portstatu_but;
	BOOL m_bAutoSend;
	char ConvertHexData(char ch);
	int String2Hex(CString str, char *SendOut);
	int WriteBlock(char *abOut,int MaxLength);
	int ReadBlock(BYTE *abIn,int MaxLength);
	volatile int m_bConnected;
	BOOL OpenComm(int Num);
	BOOL SetCommParameter();
	HANDLE m_hCom;
	CWinThread *m_pThread;
	CFile getdatafile;
	unsigned short int Get_CRC16_Code(unsigned char* pchMsg, unsigned short int wDataLen);
	int crc16(unsigned char *puchMsg, int usDataLen);
//	volatile HANDLE m_hPostMsgEvent;
//	volatile HWND hPostToWnd;
	OVERLAPPED m_osRead,m_osWrite; 
	CRadarDlg(CWnd* pParent = NULL);	// standard constructor

// Dialog Data
	//{{AFX_DATA(CRadarDlg)
	enum { IDD = IDD_COMMTEST_DIALOG };
	CButton	m_emptyrx;
	CStatic	m_filename;
	CButton	m_cHexSend;
	CButton	m_emptytx;
	CButton	m_transfile;
	CButton	m_sendbutten;
	CEdit	m_edittxdata;
	CStatic	m_sFilePath;
	CEdit	m_EditRXData;
	CEdit	m_EditFruinfo;
	CEdit	m_EditPower;
	CButton	m_ctrlHexDisplay;
	CButton	m_judgesend;
	CComboBox	m_ComboStop;
	CComboBox	m_ComboJiaoyan;
	CComboBox	m_ComboData;
	CComboBox	m_ComboBaud;
	CComboBox	m_ComboSeriou;
	CComboBox	m_ComboWorkerstatus;
	CComboBox	m_ComboReaddistance;
	CComboBox	m_ComboBuzzer;
	CString	m_SeriouStr;
	CString	m_BaudStr;
	CString	m_StopStr;
	CString	m_savefilename;
	CString	m_strTXData;
	CString	m_filepath;
	CString	m_strRXData;
	CString	m_JiaoyanStr;
	CString	m_DataStr;
	CString m_NULL;
	CString m_displayenter;
	CString m_strcrcData;
	CString m_senddateall;
	CString	m_Buzzerstr;
	CString	m_Workerstatustr;
	CString m_Readdistancestr;
	CString	m_Fruinfostr;
	CString	m_Powerstr;
	CString m_sendworkerstatustr;
	CString m_sendreaddistancestr;
	CString m_sendbuzzerstr;
	CString m_sendfruinfostr;
	CString m_sendpowerstr;
	INT		m_sendworks;
	INT		m_sendreadd;
	INT		m_sendbuzz;
	INT		m_sendfru;
	INT		m_sendpow;
	CString m_framehead;
	CString	m_framesequence;
	CString	m_framenull;
	int m_counters;
	CString	m_stradirectortodisplay,m_strdirectortodisplay,m_strdienabletodisplay,m_strstatetodisplay,m_strmoduletodisplay;
	INT	m_dvtodisplay;
	INT	m_dchannel1todisplay;
	INT	m_dchannel2todisplay;
	DOUBLE	m_dacceleratortodisplay;

	//}}AFX_DATA

	// ClassWizard generated virtual function overrides
	//{{AFX_VIRTUAL(CRadarDlg)
	protected:
	virtual void DoDataExchange(CDataExchange* pDX);	// DDX/DDV support
	//}}AFX_VIRTUAL

// Implementation
protected:
	HICON m_hIcon;

	// Generated message map functions
	//{{AFX_MSG(CRadarDlg)
	virtual BOOL OnInitDialog();
	afx_msg void OnSysCommand(UINT nID, LPARAM lParam);
	afx_msg void OnPaint();
	afx_msg HCURSOR OnQueryDragIcon();
	afx_msg void OnSelchangeComboSeriou();
	afx_msg void OnButtenMenualsend();
	afx_msg void OnButtonTransfile();
	afx_msg void OnCheckAutosend();
	afx_msg void OnTimer(UINT nIDEvent);
	afx_msg void OnButtonEmptytext();
	afx_msg void OnCheckWritetofile();
	afx_msg void OnButtonFilename();
	afx_msg void OnButtonSaveedit();
	afx_msg void OnButtonEmptyrx();
	afx_msg void OnButtonPortstatu();
	afx_msg void OnButtonClose();
	afx_msg void OnSelchangeComboBaud();
	afx_msg void OnSelchangeComboData();
	afx_msg void OnSelchangeComboJiaoyan();
	afx_msg void OnSelchangeComboStop();
	afx_msg void OnSelchangeComboWorkstatus();
	afx_msg void OnSelchangeComboReaddistance();
	afx_msg void OnSelchangeComboBuzzer();
	afx_msg void OnSize(UINT nType, int cx, int cy);
	afx_msg void OnButtonReset();
	//}}AFX_MSG

	DECLARE_MESSAGE_MAP()
private:
	void addall();
};

//{{AFX_INSERT_LOCATION}}
// Microsoft Visual C++ will insert additional declarations immediately before the previous line.

#endif // !defined(AFX_COMMTESTDLG_H__094EF006_CBE0_11D7_B4F9_00E04C74763F__INCLUDED_)
