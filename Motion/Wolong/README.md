# 臥龍 PDO 25/15 bytes 適配版本

已按使用者提供的欄位順序完成 PDO 收送與欄位轉換。保留眾辰 Type20，新增 TYPE_WOLONG=21（僅本專案的擬定值；部署前核對 Motion/PLC/HMI 未佔用）。

## 必須設定
MotEcat/WolongPdo.h 中填入實機 ESI 的 WOLONG_VENDOR_ID 與 WOLONG_PRODUCT_CODE，再設 WOLONG_IDENTIFICATION_ENABLED=1。預設為0，因此未填ID時不會自動把設備當成臥龍，這不是已可直接使能運轉的整機版本。不要只用25/15長度辨識廠家。多個Revision需各自核對 mapping。

## Tx byte offsets
603F 0(2)、6041 2(2)、6061 4(1)、6064 5(4)、60B9 9(2)、60BA 11(4)、60F4 15(4)、60FD 19(4)、6077 23(2)，總共25 bytes。

## Rx byte offsets
6040 0(2)、6060 2(1)、6072 3(2)、607A 5(4)、60B8 9(2)、60FF 11(4)，總共15 bytes。

輸入status特別從byte2取出，避免現行通用路徑誤把byte0錯誤碼當status。Rx全部15 bytes由明確欄位寫入；本映射沒有目標扭矩、VFF或數位輸出命令，不會把眾辰byte18的VFF寫入15-byte資料。

## 修改位置
MotEcat.cpp：include、block_info_input、block_info_output、MotAppPrepare、mot_r0_process讀寫與status分支。新增WolongPdo.h並列入vcproj。修改行號.txt與diff供對照。整包原始碼保持CP950/CRLF，新header為ASCII/CRLF。

## 實作檢查
在任何PDO複製前驗證臥龍in/out bit offset為byte對齊、dwPdSizeIn=200 bits、dwPdSizeOut=120 bits及byte起點不超過現行WORD可表示範圍。memcpy解碼/編碼避免byte5等未對齊int指標解參考。本實作針對Windows CE EV355 x86 little-endian；不是跨端序版。

長度不符時只回報並取消該輪軟體更新；這不是驅動器安全停機，也不保證輸出清零。需在整機層實作通訊異常停機/復原。此ZIP沒有Master image總長度介面，所以只驗證slave descriptor及本地25/15長度，不能宣稱已完整驗證總image邊界。

## 驗證與限制
附tests/wolong_pdo_test.cpp，可在host上驗證每個offset、有號位置/扭矩、輸出sentinel及布局條件。測試不等於VS2008/EV355 rebuild或實機通訊；外部AtEthercat.h與SDK未包含，需補齊原環境。

沒有修改眾辰SDO、速度單位、主軸定向或Motion type20控制字；臥龍type21不會自動走Motion Type20或Type4/9旋轉分支。CSV/CSP支援、速度尺度、controlword/status廠商bit、主軸定向、encoder回繞與絕對清除須另依臥龍手冊整合。PDO相同於Panasonic Type4僅代表此byte布局相同，並不保證行為相同。

iHwCmdPos以iLastCmdPos代填，iCmdPosRet=0，因Tx未提供這两個物件；不能當驅動器回授。共用記憶體的欄位和服務沒有改變。

請先在離線/台架驗證：rebuild→正確辨識→ENI匹配→OP→只讀位置/status/模式→控制字/模式/單位核對→受控運轉。無Vendor/Product不應直接啟用。
