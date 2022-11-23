using Mono.Cecil;
using System;
using System.Linq;
using System.Reflection.Metadata;
using System.Text;
using System.Text.RegularExpressions;

namespace SkiaExtractor
{
    class Program
    {
        private static readonly Regex sanitizeName = new Regex(@"[^a-zA-Z0-9_.]", RegexOptions.Compiled);

        static void Main(string[] args)
        {
            var module = ReadModule(@"Evergine.BulletSharpPInvoke.dll");

            var t = module.MainModule.GetType("BulletSharp.UnsafeNativeMethods");

            var allSignatures = t.Methods.Select(m => (m, signature: GetSignature(m))).ToArray();
            var distictSignatures = allSignatures.Distinct().OrderBy(s => s.signature);

            StringBuilder sb = new StringBuilder();

            foreach (var sig in distictSignatures.Select(s => s.signature).Except(monoSignatures).OrderBy(s => s))
            {
                string s = $"{sig} ->\t\t{GetSuggestedSignatures(sig)}";
                sb.AppendLine(s);
                Console.WriteLine(s);
            }

            sb.AppendLine();
            sb.AppendLine();
            sb.AppendLine();
            sb.AppendLine();
            Console.WriteLine();
            foreach (var method in allSignatures.Where(s => !monoSignatures.Contains(s.signature)))
            {
                string m = $"{method.signature} :\n{method.m}\n-Suggestions: {GetSuggestedSignatures(method.signature)}";

                sb.AppendLine(m);
                sb.AppendLine();
                Console.WriteLine(m);
                Console.WriteLine();
            }

            File.WriteAllText("BulletSharpWASMMissingSignatures.txt", sb.ToString());
        }

        private static string GetSuggestedSignatures(string sig)
        {
            var returnSig = sig[0];
            var arguments = sig.Substring(1).ToCharArray();
            var suggestions = monoSignatures.Where(s =>
            {
                var targetReturnSig = s[0];

                if (targetReturnSig == returnSig)
                {
                    var targetArguments = s.Substring(1).ToCharArray().ToList();

                    foreach (var arg in arguments)
                    {
                        if (targetArguments.Contains(arg))
                        {
                            targetArguments.Remove(arg);
                        }
                        else
                        {
                            return false;
                        }
                    }

                    return true;
                }
                else
                {
                    return false;
                }
            })
            .OrderBy( a => { return a.Length; })
            .ToList();

            if (suggestions.Count == 0)
            {
                return "NO SUGGESTIONS!!!";
            }

            StringBuilder sigSb = new StringBuilder();
            foreach (var suggestion in suggestions)
            {
                sigSb.Append($"{suggestion}, ");
            }

            return sigSb.ToString();
        }

        private static string GetSignature(Mono.Cecil.MethodDefinition d)
        {
            string GetParameterSignature(ParameterDefinition p)
            {
                if (p.ParameterType.IsByReference
                    || p.ParameterType.IsArray
                    || p.ParameterType.IsPointer)
                {
                    return "I";
                }

                return GetTypeSignature(p.ParameterType.Resolve());
            }

            string GetTypeSignature(Mono.Cecil.TypeDefinition p)
            {
                if (p.IsEnum || p.IsPointer || p.IsArray)
                {
                    return "I";
                }

                switch (p.FullName)
                {
                    case "SkiaSharp.SKPMColor":

                    case "System.String":
                    case "System.UInt16":
                    case "System.UInt32":
                    case "System.Int32":
                    case "System.IntPtr":
                    case "System.Byte":
                    case "System.Boolean":
                    case "System.Void*":
                        return "I";

                    case "System.Double":
                        return "D";

                    case "System.Single":
                        return "F";

                    case "System.Void":
                        return "V";

                    case "System.Int64":
                    case "System.UInt64":
                        return "L";

                    default:
                        throw new NotSupportedException($"{d}: {p}");
                }
            }

            return GetTypeSignature(d.ReturnType?.Resolve()) + string.Join("", d.Parameters.Select(GetParameterSignature));
        }


        private static Mono.Cecil.AssemblyDefinition ReadModule(string path)
        {
            var resolver = new DefaultAssemblyResolver();

            return Mono.Cecil.AssemblyDefinition.ReadAssembly(path, new ReaderParameters() { AssemblyResolver = resolver });
        }

        // Get the list from https://github.com/mono/mono/blob/8ced549d0625697d4a1c4d42eb1c8515553fbf7c/mono/mini/m2n-gen.cs
        static string[] monoSignatures = new string[] {
        "V",
        "VI",
        "VII",
        "VIII",
        "VIIII",
        "VIIIII",
        "VIIIIII",
        "VIIIIIII",
        "VIIIIIIII",
        "VIIIIIIIII",
        "VIIIIIIIIII",
        "VIIIIIIIIIII",
        "VIIIIIIIIIIII",
        "VIIIIIIIIIIIII",
        "VIIIIIIIIIIIIII",
        "VIIIIIIIIIIIIIII",
        "I",
        "II",
        "III",
        "IIII",
        "IIIII",
        "IIIIII",
        "IIIIIII",
        "IIIIIIII",
        "IIIIIIIII",
        "IIIIIIIIII",
        "IIIIIIIIIII",
        "IIIIIIIIIIII",
        "IIIIIIIIIIIII",
        "IIIIIIIIIIIIII",
        "IILIIII",
        "IIIL",
        "IF",
        "ID",
        "IIF",
        "IIFI",
        "IIFF",
        "IFFII",
        "IIFII",
        "IIFFI",
        "IIFFF",
        "IIFFFI",
        "IIFFII",
        "IIFIII",
        "IIFFFFI",
        "IIFFFFII",
        "IIIF",
        "IIIFI",
        "IIIFII",
        "IIIFIII",
        "IIIIF",
        "IIIIFI",
        "IIIIFII",
        "IIIIFIII",
        "IIIFFFF",
        "IIIFFFFF",
        "IIFFFFFF",
        "IIIFFFFFF",
        "IIIIIIIF",
        "IIIIIIIFF",
        "IIFFFFFFFF",
        "IIIFFFFFFFF",
        "IIIIIIFII",
        "IIIFFFFFFFFIII",
        "IIIIIFFFFIIII",
        "IFFFFFFI",
        "IIFFIII",
        "ILI",
        "IILLI",
        "L",
        "LL",
        "LI",
        "LIL",
        "LILI",
        "LILII",
        "DD",
        "DDI",
        "DDD",
        "DDDD",
        "VF",
        "VFF",
        "VFFF",
        "VFFFF",
        "VFFFFF",
        "VFFFFFF",
        "VFFFFFFF",
        "VFFFFFFFF",
        "VFI",
        "VIF",
        "VIFF",
        "VIFFFF",
        "VIFFFFF",
        "VIFFFFFF",
        "VIFFFFFI",
        "VIIFFI",
        "VIIF",
        "VIIFFF",
        "VIIFI",
        "FF",
        "FFI",
        "FFF",
        "FFFF",
        "DI",
        "FI",
        "IIL",
        "IILI",
        "IILIIIL",
        "IILLLI",
        "IDIII",
        "LII",
        "VID",
        "VILLI",
        "DID",
        "DIDD",
        "FIF",
        "FIFF",
        "LILL",
        "VL",
        "VIL",
        "VIIL",
        "FIFFF",
        "FII",
        "FIII",
        "FIIIIII",
        "IFFFFIIII",
        "IFFI",
        "IFFIF",
        "IFFIFI",
        "IFI",
        "IFIII",
        "IIFIFIIIII",
        "IIFIFIIIIII",
        "IIFIIIII",
        "IIFIIIIII",
        "IIIFFFII",
        "IIIFFIFFFII",
        "IIIFFIFFII",
        "IIIFFII",
        "IIIFFIIIII",
        "IIIIIF",
        "IIIIIFII",
        "IIIIIIFFI",
        "IIIIIIIFFI",
        "VIFFF",
        "VIFFFFI",
        "VIFFFI",
        "VIFFFIIFF",
        "VIFFI",
        "VIFI",
        "VIIFF",
        "VIIFFFF",
        "VIIFFII",
        "VIIIF",
        "VIIIFFII",
        "VIIIFFIII",
        "VIIIFII",
        "VIIIFIII",
        "VIIIIF",
        "IFFFFIII",
        "IFFIII",
        "VIIIIFFII",
        "IIILIIII",
        "IIILLI",
        "IL",
        "IFF",
        "IFFF",
        "IFFFF",
        "VLII",
        "IIIIL",
        "LIIIL",
        "IILL",
    };
    }
}