#include "GetFileName.h"

void GetFileFmat(LPCTSTR path, vector <wstring > &filesPathVector, TCHAR *fmat)
{
//find the first file
_tfinddata64_t c_file;
intptr_t hFile;
TCHAR root[MAX_PATH];
_tcscpy_s(root, MAX_PATH, path);
_tcscat_s(root, MAX_PATH, _T("\\*"));
_tcscat_s(root, MAX_PATH, fmat);
hFile = _tfindfirst64(root, &c_file);
if (hFile == -1)
return;

//search all files recursively.
do
{
	if (_tcslen(c_file.name) == 1 && c_file.name[0] == _T('.')
		|| _tcslen(c_file.name) == 2 && c_file.name[0] == _T('.') && c_file.name[1] == _T('.'))
		continue;
	TCHAR *fullPath = new TCHAR[MAX_PATH];
	_tcscpy_s(fullPath, MAX_PATH, path);
	_tcscat_s(fullPath, MAX_PATH, _T("\\"));
	_tcscat_s(fullPath, MAX_PATH, c_file.name);
	if (c_file.attrib&_A_SUBDIR)
	{
		GetFileFmat(fullPath, filesPathVector, fmat);
	}
	else
	{
		//store full file path in vector.
		filesPathVector.push_back(fullPath);
	}
	delete[] fullPath;
} while (_tfindnext64(hFile, &c_file) == 0);
//close search handle
_findclose(hFile);
}


//��þ���·���е�����ļ���name��������path
//�β�nameΪ�洢�ļ����������׵�ַ
void get_name_from_pathname(char * const ppathname, char* name)
{
	char* ppath_tmp = ppathname;
	char* last_p2backslash = nullptr;
	char* p2point = nullptr;
	char* p2slash = nullptr;
	int i = 0;
	do
	{
		if (*ppath_tmp == '\\')	last_p2backslash = ppath_tmp;
		if (*ppath_tmp == '.')		p2point = ppath_tmp;
		if (*ppath_tmp == '-') {
			p2slash = ppath_tmp;
		}
		ppath_tmp++;
	} while (*ppath_tmp != '\0');
	char* pname = name;
	ppath_tmp = (p2slash + 1);
	do
	{
		*pname++ = *ppath_tmp++;

	} while (ppath_tmp < p2point);
}

//void THCAR2char(TCHAR* tchStr, char chRtn[MAX_PATH])
//{
//	char *pCHAR = chRtn;
//	int iLen = 2 * wcslen(tchStr);//CString,TCHAR������һ���ַ�����˲�����ͨ���㳤�� 
//								  //	char* chRtn = new char[iLen + 1];
//	size_t J;
//	wcstombs_s(&J, chRtn, MAX_PATH, tchStr, _tcslen(tchStr) + 1);//ת���ɹ�����Ϊ�Ǹ�ֵ 
//}

//��TCHARתΪchar   
//*tchar��TCHAR����ָ�룬*_char��char����ָ��   
void TcharToChar(const TCHAR * tchar, char * _char)
{
	int iLength;
	//��ȡ�ֽڳ���   
	iLength = WideCharToMultiByte(CP_ACP, 0, tchar, -1, NULL, 0, NULL, NULL);
	//��tcharֵ����_char    
	WideCharToMultiByte(CP_ACP, 0, tchar, -1, _char, iLength, NULL, NULL);
}