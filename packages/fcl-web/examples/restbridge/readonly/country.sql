CREATE TABLE country (
  iso character(2) NOT NULL,
  name character varying(80) NOT NULL,
  nicename character varying(80) NOT NULL,
  iso3 character(3) DEFAULT NULL::bpchar,
  numcode smallint,
  phonecode smallint NOT NULL,
  CONSTRAINT country_pkey PRIMARY KEY (iso)
);

INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('AF', 'AFGHANISTAN', 'Afghanistan', 'AFG', 4, 93);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('AL', 'ALBANIA', 'Albania', 'ALB', 8, 355);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('DZ', 'ALGERIA', 'Algeria', 'DZA', 12, 213);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('AS', 'AMERICAN SAMOA', 'American Samoa', 'ASM', 16, 1684);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('AD', 'ANDORRA', 'Andorra', 'AND', 20, 376);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('AO', 'ANGOLA', 'Angola', 'AGO', 24, 244);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('AI', 'ANGUILLA', 'Anguilla', 'AIA', 660, 1264);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('AQ', 'ANTARCTICA', 'Antarctica', '', , 0);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('AG', 'ANTIGUA AND BARBUDA', 'Antigua and Barbuda', 'ATG', 28, 1268);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('AR', 'ARGENTINA', 'Argentina', 'ARG', 32, 54);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('AM', 'ARMENIA', 'Armenia', 'ARM', 51, 374);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('AW', 'ARUBA', 'Aruba', 'ABW', 533, 297);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('AU', 'AUSTRALIA', 'Australia', 'AUS', 36, 61);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('AT', 'AUSTRIA', 'Austria', 'AUT', 40, 43);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('AZ', 'AZERBAIJAN', 'Azerbaijan', 'AZE', 31, 994);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('BS', 'BAHAMAS', 'Bahamas', 'BHS', 44, 1242);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('BH', 'BAHRAIN', 'Bahrain', 'BHR', 48, 973);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('BD', 'BANGLADESH', 'Bangladesh', 'BGD', 50, 880);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('BB', 'BARBADOS', 'Barbados', 'BRB', 52, 1246);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('BY', 'BELARUS', 'Belarus', 'BLR', 112, 375);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('BE', 'BELGIUM', 'Belgium', 'BEL', 56, 32);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('BZ', 'BELIZE', 'Belize', 'BLZ', 84, 501);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('BJ', 'BENIN', 'Benin', 'BEN', 204, 229);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('BM', 'BERMUDA', 'Bermuda', 'BMU', 60, 1441);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('BT', 'BHUTAN', 'Bhutan', 'BTN', 64, 975);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('BO', 'BOLIVIA', 'Bolivia', 'BOL', 68, 591);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('BA', 'BOSNIA AND HERZEGOVINA', 'Bosnia and Herzegovina', 'BIH', 70, 387);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('BW', 'BOTSWANA', 'Botswana', 'BWA', 72, 267);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('BV', 'BOUVET ISLAND', 'Bouvet Island', '', , 0);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('BR', 'BRAZIL', 'Brazil', 'BRA', 76, 55);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('IO', 'BRITISH INDIAN OCEAN TERRITORY', 'British Indian Ocean Territory', '', , 246);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('BN', 'BRUNEI DARUSSALAM', 'Brunei Darussalam', 'BRN', 96, 673);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('BG', 'BULGARIA', 'Bulgaria', 'BGR', 100, 359);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('BF', 'BURKINA FASO', 'Burkina Faso', 'BFA', 854, 226);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('BI', 'BURUNDI', 'Burundi', 'BDI', 108, 257);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('KH', 'CAMBODIA', 'Cambodia', 'KHM', 116, 855);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('CM', 'CAMEROON', 'Cameroon', 'CMR', 120, 237);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('CA', 'CANADA', 'Canada', 'CAN', 124, 1);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('CV', 'CAPE VERDE', 'Cape Verde', 'CPV', 132, 238);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('KY', 'CAYMAN ISLANDS', 'Cayman Islands', 'CYM', 136, 1345);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('CF', 'CENTRAL AFRICAN REPUBLIC', 'Central African Republic', 'CAF', 140, 236);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('TD', 'CHAD', 'Chad', 'TCD', 148, 235);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('CL', 'CHILE', 'Chile', 'CHL', 152, 56);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('CN', 'CHINA', 'China', 'CHN', 156, 86);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('CX', 'CHRISTMAS ISLAND', 'Christmas Island', '', , 61);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('CC', 'COCOS (KEELING) ISLANDS', 'Cocos (Keeling) Islands', '', , 672);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('CO', 'COLOMBIA', 'Colombia', 'COL', 170, 57);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('KM', 'COMOROS', 'Comoros', 'COM', 174, 269);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('CG', 'CONGO', 'Congo', 'COG', 178, 242);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('CD', 'CONGO, THE DEMOCRATIC REPUBLIC OF THE', 'Congo, the Democratic Republic of the', 'COD', 180, 242);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('CK', 'COOK ISLANDS', 'Cook Islands', 'COK', 184, 682);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('CR', 'COSTA RICA', 'Costa Rica', 'CRI', 188, 506);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('CI', 'COTE D''IVOIRE', 'Cote D''Ivoire', 'CIV', 384, 225);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('HR', 'CROATIA', 'Croatia', 'HRV', 191, 385);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('CU', 'CUBA', 'Cuba', 'CUB', 192, 53);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('CY', 'CYPRUS', 'Cyprus', 'CYP', 196, 357);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('CZ', 'CZECH REPUBLIC', 'Czech Republic', 'CZE', 203, 420);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('DK', 'DENMARK', 'Denmark', 'DNK', 208, 45);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('DJ', 'DJIBOUTI', 'Djibouti', 'DJI', 262, 253);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('DM', 'DOMINICA', 'Dominica', 'DMA', 212, 1767);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('DO', 'DOMINICAN REPUBLIC', 'Dominican Republic', 'DOM', 214, 1809);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('EC', 'ECUADOR', 'Ecuador', 'ECU', 218, 593);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('EG', 'EGYPT', 'Egypt', 'EGY', 818, 20);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('SV', 'EL SALVADOR', 'El Salvador', 'SLV', 222, 503);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('GQ', 'EQUATORIAL GUINEA', 'Equatorial Guinea', 'GNQ', 226, 240);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('ER', 'ERITREA', 'Eritrea', 'ERI', 232, 291);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('EE', 'ESTONIA', 'Estonia', 'EST', 233, 372);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('ET', 'ETHIOPIA', 'Ethiopia', 'ETH', 231, 251);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('FK', 'FALKLAND ISLANDS (MALVINAS)', 'Falkland Islands (Malvinas)', 'FLK', 238, 500);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('FO', 'FAROE ISLANDS', 'Faroe Islands', 'FRO', 234, 298);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('FJ', 'FIJI', 'Fiji', 'FJI', 242, 679);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('FI', 'FINLAND', 'Finland', 'FIN', 246, 358);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('FR', 'FRANCE', 'France', 'FRA', 250, 33);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('GF', 'FRENCH GUIANA', 'French Guiana', 'GUF', 254, 594);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('PF', 'FRENCH POLYNESIA', 'French Polynesia', 'PYF', 258, 689);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('TF', 'FRENCH SOUTHERN TERRITORIES', 'French Southern Territories', '', , 0);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('GA', 'GABON', 'Gabon', 'GAB', 266, 241);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('GM', 'GAMBIA', 'Gambia', 'GMB', 270, 220);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('GE', 'GEORGIA', 'Georgia', 'GEO', 268, 995);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('DE', 'GERMANY', 'Germany', 'DEU', 276, 49);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('GH', 'GHANA', 'Ghana', 'GHA', 288, 233);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('GI', 'GIBRALTAR', 'Gibraltar', 'GIB', 292, 350);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('GR', 'GREECE', 'Greece', 'GRC', 300, 30);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('GL', 'GREENLAND', 'Greenland', 'GRL', 304, 299);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('GD', 'GRENADA', 'Grenada', 'GRD', 308, 1473);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('GP', 'GUADELOUPE', 'Guadeloupe', 'GLP', 312, 590);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('GU', 'GUAM', 'Guam', 'GUM', 316, 1671);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('GT', 'GUATEMALA', 'Guatemala', 'GTM', 320, 502);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('GN', 'GUINEA', 'Guinea', 'GIN', 324, 224);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('GW', 'GUINEA-BISSAU', 'Guinea-Bissau', 'GNB', 624, 245);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('GY', 'GUYANA', 'Guyana', 'GUY', 328, 592);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('HT', 'HAITI', 'Haiti', 'HTI', 332, 509);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('HM', 'HEARD ISLAND AND MCDONALD ISLANDS', 'Heard Island and Mcdonald Islands', '', , 0);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('VA', 'HOLY SEE (VATICAN CITY STATE)', 'Holy See (Vatican City State)', 'VAT', 336, 39);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('HN', 'HONDURAS', 'Honduras', 'HND', 340, 504);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('HK', 'HONG KONG', 'Hong Kong', 'HKG', 344, 852);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('HU', 'HUNGARY', 'Hungary', 'HUN', 348, 36);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('IS', 'ICELAND', 'Iceland', 'ISL', 352, 354);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('IN', 'INDIA', 'India', 'IND', 356, 91);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('ID', 'INDONESIA', 'Indonesia', 'IDN', 360, 62);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('IR', 'IRAN, ISLAMIC REPUBLIC OF', 'Iran, Islamic Republic of', 'IRN', 364, 98);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('IQ', 'IRAQ', 'Iraq', 'IRQ', 368, 964);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('IE', 'IRELAND', 'Ireland', 'IRL', 372, 353);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('IL', 'ISRAEL', 'Israel', 'ISR', 376, 972);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('IT', 'ITALY', 'Italy', 'ITA', 380, 39);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('JM', 'JAMAICA', 'Jamaica', 'JAM', 388, 1876);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('JP', 'JAPAN', 'Japan', 'JPN', 392, 81);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('JO', 'JORDAN', 'Jordan', 'JOR', 400, 962);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('KZ', 'KAZAKHSTAN', 'Kazakhstan', 'KAZ', 398, 7);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('KE', 'KENYA', 'Kenya', 'KEN', 404, 254);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('KI', 'KIRIBATI', 'Kiribati', 'KIR', 296, 686);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('KP', 'KOREA, DEMOCRATIC PEOPLE''S REPUBLIC OF', 'Korea, Democratic People''s Republic of', 'PRK', 408, 850);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('KR', 'KOREA, REPUBLIC OF', 'Korea, Republic of', 'KOR', 410, 82);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('KW', 'KUWAIT', 'Kuwait', 'KWT', 414, 965);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('KG', 'KYRGYZSTAN', 'Kyrgyzstan', 'KGZ', 417, 996);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('LA', 'LAO PEOPLE''S DEMOCRATIC REPUBLIC', 'Lao People''s Democratic Republic', 'LAO', 418, 856);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('LV', 'LATVIA', 'Latvia', 'LVA', 428, 371);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('LB', 'LEBANON', 'Lebanon', 'LBN', 422, 961);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('LS', 'LESOTHO', 'Lesotho', 'LSO', 426, 266);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('LR', 'LIBERIA', 'Liberia', 'LBR', 430, 231);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('LY', 'LIBYAN ARAB JAMAHIRIYA', 'Libyan Arab Jamahiriya', 'LBY', 434, 218);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('LI', 'LIECHTENSTEIN', 'Liechtenstein', 'LIE', 438, 423);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('LT', 'LITHUANIA', 'Lithuania', 'LTU', 440, 370);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('LU', 'LUXEMBOURG', 'Luxembourg', 'LUX', 442, 352);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('MO', 'MACAO', 'Macao', 'MAC', 446, 853);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('MK', 'MACEDONIA, THE FORMER YUGOSLAV REPUBLIC OF', 'Macedonia, the Former Yugoslav Republic of', 'MKD', 807, 389);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('MG', 'MADAGASCAR', 'Madagascar', 'MDG', 450, 261);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('MW', 'MALAWI', 'Malawi', 'MWI', 454, 265);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('MY', 'MALAYSIA', 'Malaysia', 'MYS', 458, 60);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('MV', 'MALDIVES', 'Maldives', 'MDV', 462, 960);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('ML', 'MALI', 'Mali', 'MLI', 466, 223);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('MT', 'MALTA', 'Malta', 'MLT', 470, 356);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('MH', 'MARSHALL ISLANDS', 'Marshall Islands', 'MHL', 584, 692);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('MQ', 'MARTINIQUE', 'Martinique', 'MTQ', 474, 596);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('MR', 'MAURITANIA', 'Mauritania', 'MRT', 478, 222);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('MU', 'MAURITIUS', 'Mauritius', 'MUS', 480, 230);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('YT', 'MAYOTTE', 'Mayotte', '', , 269);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('MX', 'MEXICO', 'Mexico', 'MEX', 484, 52);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('FM', 'MICRONESIA, FEDERATED STATES OF', 'Micronesia, Federated States of', 'FSM', 583, 691);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('MD', 'MOLDOVA, REPUBLIC OF', 'Moldova, Republic of', 'MDA', 498, 373);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('MC', 'MONACO', 'Monaco', 'MCO', 492, 377);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('MN', 'MONGOLIA', 'Mongolia', 'MNG', 496, 976);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('MS', 'MONTSERRAT', 'Montserrat', 'MSR', 500, 1664);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('MA', 'MOROCCO', 'Morocco', 'MAR', 504, 212);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('MZ', 'MOZAMBIQUE', 'Mozambique', 'MOZ', 508, 258);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('MM', 'MYANMAR', 'Myanmar', 'MMR', 104, 95);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('NA', 'NAMIBIA', 'Namibia', 'NAM', 516, 264);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('NR', 'NAURU', 'Nauru', 'NRU', 520, 674);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('NP', 'NEPAL', 'Nepal', 'NPL', 524, 977);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('NL', 'NETHERLANDS', 'Netherlands', 'NLD', 528, 31);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('AN', 'NETHERLANDS ANTILLES', 'Netherlands Antilles', 'ANT', 530, 599);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('NC', 'NEW CALEDONIA', 'New Caledonia', 'NCL', 540, 687);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('NZ', 'NEW ZEALAND', 'New Zealand', 'NZL', 554, 64);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('NI', 'NICARAGUA', 'Nicaragua', 'NIC', 558, 505);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('NE', 'NIGER', 'Niger', 'NER', 562, 227);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('NG', 'NIGERIA', 'Nigeria', 'NGA', 566, 234);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('NU', 'NIUE', 'Niue', 'NIU', 570, 683);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('NF', 'NORFOLK ISLAND', 'Norfolk Island', 'NFK', 574, 672);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('MP', 'NORTHERN MARIANA ISLANDS', 'Northern Mariana Islands', 'MNP', 580, 1670);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('NO', 'NORWAY', 'Norway', 'NOR', 578, 47);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('OM', 'OMAN', 'Oman', 'OMN', 512, 968);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('PK', 'PAKISTAN', 'Pakistan', 'PAK', 586, 92);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('PW', 'PALAU', 'Palau', 'PLW', 585, 680);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('PS', 'PALESTINIAN TERRITORY, OCCUPIED', 'Palestinian Territory, Occupied', '', , 970);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('PA', 'PANAMA', 'Panama', 'PAN', 591, 507);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('PG', 'PAPUA NEW GUINEA', 'Papua New Guinea', 'PNG', 598, 675);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('PY', 'PARAGUAY', 'Paraguay', 'PRY', 600, 595);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('PE', 'PERU', 'Peru', 'PER', 604, 51);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('PH', 'PHILIPPINES', 'Philippines', 'PHL', 608, 63);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('PN', 'PITCAIRN', 'Pitcairn', 'PCN', 612, 0);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('PL', 'POLAND', 'Poland', 'POL', 616, 48);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('PT', 'PORTUGAL', 'Portugal', 'PRT', 620, 351);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('PR', 'PUERTO RICO', 'Puerto Rico', 'PRI', 630, 1787);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('QA', 'QATAR', 'Qatar', 'QAT', 634, 974);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('RE', 'REUNION', 'Reunion', 'REU', 638, 262);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('RO', 'ROMANIA', 'Romania', 'ROM', 642, 40);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('RU', 'RUSSIAN FEDERATION', 'Russian Federation', 'RUS', 643, 70);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('RW', 'RWANDA', 'Rwanda', 'RWA', 646, 250);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('SH', 'SAINT HELENA', 'Saint Helena', 'SHN', 654, 290);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('KN', 'SAINT KITTS AND NEVIS', 'Saint Kitts and Nevis', 'KNA', 659, 1869);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('LC', 'SAINT LUCIA', 'Saint Lucia', 'LCA', 662, 1758);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('PM', 'SAINT PIERRE AND MIQUELON', 'Saint Pierre and Miquelon', 'SPM', 666, 508);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('VC', 'SAINT VINCENT AND THE GRENADINES', 'Saint Vincent and the Grenadines', 'VCT', 670, 1784);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('WS', 'SAMOA', 'Samoa', 'WSM', 882, 684);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('SM', 'SAN MARINO', 'San Marino', 'SMR', 674, 378);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('ST', 'SAO TOME AND PRINCIPE', 'Sao Tome and Principe', 'STP', 678, 239);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('SA', 'SAUDI ARABIA', 'Saudi Arabia', 'SAU', 682, 966);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('SN', 'SENEGAL', 'Senegal', 'SEN', 686, 221);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('CS', 'SERBIA AND MONTENEGRO', 'Serbia and Montenegro', '', , 381);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('SC', 'SEYCHELLES', 'Seychelles', 'SYC', 690, 248);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('SL', 'SIERRA LEONE', 'Sierra Leone', 'SLE', 694, 232);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('SG', 'SINGAPORE', 'Singapore', 'SGP', 702, 65);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('SK', 'SLOVAKIA', 'Slovakia', 'SVK', 703, 421);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('SI', 'SLOVENIA', 'Slovenia', 'SVN', 705, 386);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('SB', 'SOLOMON ISLANDS', 'Solomon Islands', 'SLB', 90, 677);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('SO', 'SOMALIA', 'Somalia', 'SOM', 706, 252);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('ZA', 'SOUTH AFRICA', 'South Africa', 'ZAF', 710, 27);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('GS', 'SOUTH GEORGIA AND THE SOUTH SANDWICH ISLANDS', 'South Georgia and the South Sandwich Islands', '', , 0);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('ES', 'SPAIN', 'Spain', 'ESP', 724, 34);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('LK', 'SRI LANKA', 'Sri Lanka', 'LKA', 144, 94);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('SD', 'SUDAN', 'Sudan', 'SDN', 736, 249);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('SR', 'SURINAME', 'Suriname', 'SUR', 740, 597);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('SJ', 'SVALBARD AND JAN MAYEN', 'Svalbard and Jan Mayen', 'SJM', 744, 47);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('SZ', 'SWAZILAND', 'Swaziland', 'SWZ', 748, 268);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('SE', 'SWEDEN', 'Sweden', 'SWE', 752, 46);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('CH', 'SWITZERLAND', 'Switzerland', 'CHE', 756, 41);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('SY', 'SYRIAN ARAB REPUBLIC', 'Syrian Arab Republic', 'SYR', 760, 963);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('TW', 'TAIWAN, PROVINCE OF CHINA', 'Taiwan, Province of China', 'TWN', 158, 886);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('TJ', 'TAJIKISTAN', 'Tajikistan', 'TJK', 762, 992);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('TZ', 'TANZANIA, UNITED REPUBLIC OF', 'Tanzania, United Republic of', 'TZA', 834, 255);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('TH', 'THAILAND', 'Thailand', 'THA', 764, 66);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('TL', 'TIMOR-LESTE', 'Timor-Leste', '', , 670);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('TG', 'TOGO', 'Togo', 'TGO', 768, 228);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('TK', 'TOKELAU', 'Tokelau', 'TKL', 772, 690);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('TO', 'TONGA', 'Tonga', 'TON', 776, 676);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('TT', 'TRINIDAD AND TOBAGO', 'Trinidad and Tobago', 'TTO', 780, 1868);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('TN', 'TUNISIA', 'Tunisia', 'TUN', 788, 216);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('TR', 'TURKEY', 'Turkey', 'TUR', 792, 90);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('TM', 'TURKMENISTAN', 'Turkmenistan', 'TKM', 795, 7370);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('TC', 'TURKS AND CAICOS ISLANDS', 'Turks and Caicos Islands', 'TCA', 796, 1649);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('TV', 'TUVALU', 'Tuvalu', 'TUV', 798, 688);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('UG', 'UGANDA', 'Uganda', 'UGA', 800, 256);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('UA', 'UKRAINE', 'Ukraine', 'UKR', 804, 380);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('AE', 'UNITED ARAB EMIRATES', 'United Arab Emirates', 'ARE', 784, 971);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('GB', 'UNITED KINGDOM', 'United Kingdom', 'GBR', 826, 44);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('US', 'UNITED STATES', 'United States', 'USA', 840, 1);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('UM', 'UNITED STATES MINOR OUTLYING ISLANDS', 'United States Minor Outlying Islands', '', , 1);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('UY', 'URUGUAY', 'Uruguay', 'URY', 858, 598);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('UZ', 'UZBEKISTAN', 'Uzbekistan', 'UZB', 860, 998);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('VU', 'VANUATU', 'Vanuatu', 'VUT', 548, 678);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('VE', 'VENEZUELA', 'Venezuela', 'VEN', 862, 58);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('VN', 'VIET NAM', 'Viet Nam', 'VNM', 704, 84);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('VG', 'VIRGIN ISLANDS, BRITISH', 'Virgin Islands, British', 'VGB', 92, 1284);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('VI', 'VIRGIN ISLANDS, U.S.', 'Virgin Islands, U.s.', 'VIR', 850, 1340);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('WF', 'WALLIS AND FUTUNA', 'Wallis and Futuna', 'WLF', 876, 681);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('EH', 'WESTERN SAHARA', 'Western Sahara', 'ESH', 732, 212);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('YE', 'YEMEN', 'Yemen', 'YEM', 887, 967);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('ZM', 'ZAMBIA', 'Zambia', 'ZMB', 894, 260);
INSERT INTO country (iso, name, nicename, iso3, numcode, phonecode)
 VALUES ('ZW', 'ZIMBABWE', 'Zimbabwe', 'ZWE', 716, 263);
