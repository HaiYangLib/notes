## 术语和定义

| **缩略语** | **全称**                                                     | **解释**                                                     |
| ---------- | ------------------------------------------------------------ | ------------------------------------------------------------ |
| CA         | Certificate Authority                                        | 证书认证机构，主要负责签发各种数字证书                       |
| CRL        | Certificate Revocation List                                  | 证书撤销列表，ITSCA和OEMCA均有各自独立的、不同格式的CRL      |
| V2X        | Vehicle To(2) Everything                                     | 车到一切，包括V2V/V2I/V2N/V2P                                |
| ITS        | Intelligent Transportation System                            | 智能交通系统，在本文档中也用于符合《GB∕T 37376-2019 交通运输 数字证书格式》规范的数字证书格式 |
| ITSCA      | Intelligent Transportation System CA                         | 智能交通数字证书认证系统，符合智能交通规范的CA，签发的终端实体数字证书（ITS证书）主要用于V2V、V2I的PC5通信 |
| RCA        | Root Certificate Authority                                   | 根证书机构，根CA，属于ITSCA体系                              |
| ECA        | Enrolment Certificate Authority                              | 注册证书机构，注册CA                                         |
| ACA        | Authorization Certificate AuthorityApplication Certificate Authority | 授权证书机构，授权CA，也叫做应用证书机构，应用CA             |
| PCA        | Pseudonym CA                                                 | 假名证书机构，假名CA                                         |
| CRA        | Certificate Revoke Authority                                 | 证书撤销机构，同CRLSigner                                    |
| CRLSigner  | CRL Signer                                                   | 证书撤销列表签发机构，CRL签发机构                            |
| EC         | Enrolment Certificate                                        | 注册证书                                                     |
| AC         | Authorization Certificate                                    | 应用证书                                                     |
| PC         | Pseudonym Certificate                                        | 假名证书                                                     |
| CACRL      | CA Certificate Revocation List                               | CA证书撤销列表                                               |
| ECCRL      | EC Certificate Revocation List                               | 注册证书撤销列表                                             |
| HSM        | Hardware Security Module                                     | 硬件安全芯片，具体密钥对生成、存储、运算能力                 |
| V-HSM      | Virtual-Hardware Security Module                             | 虚拟的硬件安全模块，使用软件模拟的安全模块                   |
| SM         | Security Module                                              | 安全模块，包含HSM/V-HSM                                      |
| OBU        | On Board Unit                                                | 车载V2X通信终端                                              |
| RSU        | Road Side Unit                                               | 路侧单元                                                     |
| EE         | End Entity                                                   | 终端实体，收发V2X消息、或与云通信的终端实体单元，例如TBOX、OBU |
| VASS       | V2X Application Security Service                             | V2X应用安全服务，运行在OBU、TBOX内的安全服务                 |
| VA         | V2X Application                                              | V2X应用，运行在OBU、TBOX内                                   |
| SSL        | Secure Socket Layer                                          | 安全套接字层，兼容SSL和TLS（本文统称SSL）                    |
| GMSSL      | Guo Mi Secure Socket Layer                                   | 国密安全套接字层，支持国密SM2/SM3/SM4算法，同时也支持通用算法，如NIST等 |
| OER        | Specification of Octet Encoding Rules                        | 由ISO / IEC和ITU-T制定的一种新型，高效一套ASN.1编码规则的技术，普遍应用与ITS体系 |

## **参考资料**

* 《基于LTE的车联网无线通信技术 安全证书管理系统技术要求》
* 《基于LTE网络的车联网通信安全总体技术要求》
* 《基于LTE的车联网无线通信技术总体技术要求》
* 《基于LTE的车联网无线通信技术空口技术要求》
* 《基于LTE的车联网无线通信技术网络层技术要求》
* 《基于LTE的车联网无线通信技术消息层技术要求》
* 《面向智能网联驾驶的协同互联云平台研发与示范应用--数据采集标准接口规范》
* 《GB∕T 25056-2018 信息安全技术 证书认证系统密码及其相关安全技术规范》
* IEEE 1609.2《Wireless Access in Vehicular Environments—Security Services for Applications and Management Messages》
* GB/T37376《交通运输数字证书格式》GM/T 0016-2012 《智能密码钥匙密码应用接口规范》
* GM/T 0017-2012 《智能密码钥匙密码应用接口数据格式规范》
* GM/T 0018-2012 《密码设备应用接口规范》
* GM/T 0019-2012 《通用密码服务接口规范》
* GM/T 0027-2014 《智能密码钥匙技术规范》
* GM/T 0030-2014 《服务器密码机技术规范》
* GB/T 19714-2005 公钥基础设施 证书管理协议

## V2X CA平台结构

<img src="CA 管理.assets/image-20211108224209160.png" alt="image-20211108224209160" style="zoom:50%;" />

Root CA：根证书机构（Root CA），LTE-V2X证书管理系统的信任根，负责系统根证书的管理与维护并对LTE-V2X证书机构进行注册审批。在确认LTE-V2X证书机构的合法性之后，根证书机构为其签发管理机构的数字证书，使其成为系统内的有效实体。

ICA：中间CA（Intermediate CA，ICA），位于根CA与签发注册证书、各种授权证书CA之间，用于扩展PKI体系的层级，实现CA的多级部署和多级管理。ICA所签发证书的种类取决于ICA证书的权限或由一个PKI系统的证书管理策略确定。

TRCLA：可信根证书列表管理机构（Trusted Root Certificate List Authority，TRCLA），负责签发可信根证书列表。提供可信根证书管理功能，V2X 设备可通过此功能获取可信根证书列表的签名证书，以及可信根证书列表的下载地址。

ECA：ECA注册证书机构，为V2X终端签发注册证书，接受并处理来自DCM注册系统的业务请求，可批量签发注册证书；存储用户信息、用户证书信息；存储各类管理员身份信息、管理员权限信息、业务策略信息等。

DCM：DCM认证授权系统负责V2X终端的认证和授权，接收业务请求、转发给CA系统，提供证书下载；完成证书申请、更新、加入黑名单等证书管理功能，与车联网后台的数据同步，承接身份认证、关系绑定等功能。

PCA：假名CA（Pseudonym CA，PCA）负责向OBU签发假名证书。OBU使用假名证书对其播发的主动安全消息（Basic Safety Message，BSM）进行数字签名。为避免泄露车辆行驶路径，PCA应向OBU签发多个有效期相同的假名证书；OBU依据假名证书使用策略，定期更换用于消息签名的证书。

PRA：假名证书注册机构PRA（Pseudonym Registration Authority，PRA），接收OBU的假名证书申请，对OBU提供的假名证书种子密钥进行扩展，从LA获取相应的证书链接值；基于扩展的密钥和链接值生成假名证书生成请求并发送给假名证书CA；从假名CA获取OBU的假名证书并将其发送给OBU。

LA：链接机构（Linkage Authority，LA）实现链接值供应功能，包括个体链接值和组链接值。接收PRA的链接值请求，验证请求的有效性，并生成链接值；接受MA的链接种子查询，验证请求的有效性，并返回MA所需的链接种子。在证书撤销前，链接值不会泄露多个假名证书属于同一个证书申请实体的信息。链接值支持高效撤销、简化撤销流程、优化撤销性能，并为OBU提供可信的防跟踪能力。为保证链接值的不可链接性，可基于多机构共同管理的机制保障LA的可信。

ACA：应用CA（Application CA，ACA），又称应用授权CA（application authorization CA），负责向OBU签发身份证书（Identity Certificate）、向RSU车联网设备签发应用证书（Application Certificate）。

ARA：应用RA，用于受理应用/身份证书申请请求。

MA：异常行为管理机构（Misbehavior Authority，MA）：基于V2X设备上报的异常行为检测报告，判断需要撤销的假名证书、应用证书和身份证书，并签发相应的证书撤销列表。

MRA：不当行为RA，接收MB上报请求，并转发至MA。

另外，各业务子系统，均提供各自的管理平台，包括配置管理、业务管理、权限管理、日志审计管理、证书和设备状态管理等。



## V2X 终端结构：

<img src="CA 管理.assets/image-20211108224246012.png" alt="image-20211108224246012" style="zoom:50%;" />

1) 根CA证书由RCA系统自签生成；

2) 注册CA证书、应用CA证书、假名CA证书和CRL签发者证书由RCA系统签发，这种类型的证书被称为子CA证书，用于为终端证书进行签名。

3) 注册证书由ECA系统签发，属于设备证书的一种，可用于设备的身份证明，以用来申请假名证书；

4) 应用证书由ACA系统签发，属于设备证书的一种，作为通信中的身份标识证书；

5) 假名证书由PCA系统签发，属于设备证书的一种，具有匿名特性，用于车与车通信；

6) CA证书CRL和注册证书CRL由CRLSigner系统签发，CRL文件支持各系统下载。

## 证书类型

整套V2X认证系统按照证书不同的权限以及不同的应用场景划分为八类证书，其关系如下图：

<img src="CA 管理.assets/image-20211108223115543.png" alt="image-20211108223115543" style="zoom:50%;" />

