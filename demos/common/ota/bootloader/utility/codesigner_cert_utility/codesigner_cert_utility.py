import base64
import sys, getopt

try:
	import OpenSSL
	from OpenSSL import crypto
	from OpenSSL.crypto import X509
except:
    # Any exceptions with opening the file or loading the file prints this warning.
    print("############################################################ WARNING ############################################################")
    print("# pyopenssl not installed, required for OTA code signature verification ")
    print("############################################################ WARNING ############################################################")
    sys.exit(0)
	
file_header = "/*This file is autogenerated by codesigner_cert_utility.py. */ \n"
file_header2 = "/*cert used is "
var_array_name = "const unsigned char pucCodeSignPublicKey[] = {"
var_length_name = "const unsigned int ulCodeSignPublickeyLength = " 
indentation = "    "
end = "};\n"
file=sys.stdout

def main():
    if len(sys.argv) < 3 or len(sys.argv) > 3:
        print("Usage: python codesigner_cert_utility [path to PEM encoded certificate] [path to file to generate]")
        sys.exit(0)

    try:
        # Load the pem encoded certificate file
        with open(sys.argv[1], "r") as cert_file:
            certx509 = OpenSSL.crypto.load_certificate(OpenSSL.crypto.FILETYPE_PEM, cert_file.read().encode('ASCII'))
    except:
        # Any exceptions with opening the file or loading the file prints this warning.
        print("############################################################ WARNING ############################################################")
        print("# No certificate present in " + sys.argv[1])
        print("############################################################ WARNING ############################################################")
        sys.exit(0)

    # Get the DER encoded public key
    pubkey = certx509.get_pubkey()
    pubkeybytes = OpenSSL.crypto.dump_publickey(OpenSSL.crypto.FILETYPE_ASN1,pubkey)
    # Write the public to the header file
    with open(sys.argv[2], "w") as header_file:
        header_file.write(file_header)
        header_file.write(file_header2 + sys.argv[1] + "*/\n")
        header_file.write(var_array_name)
        for count, b in enumerate(pubkeybytes):
            if count % 8 == 0:
                header_file.write("\n" + indentation)
            else:
                header_file.write(" ")
            header_file.write("0x{:02x},".format(b))
        header_file.write("\n" + indentation + end)
        header_file.write(var_length_name + str(len(pubkeybytes)) + ";")
	
if __name__ == '__main__':
    main()
