#pragma warning(disable: 4002)

#include <QtWidgets/QApplication>
#include "mainwindow.h"

//#include "test.h"
#include "guicon.h"

enum process_levels{ nada, ayuda, test, testPackage};

void showHelp(){

  printf("   >> Options:\n");
  printf("  --testpkg ARG or -p ARG          Process coords from file ARG specifications\n");
  printf("  --test  or -t                    Execute predefined test function\n");
  printf("  --help  or -h                    Show this help\n");
  printf("\n----------------------------------------------------------------------\n\n");
  QApplication::exit();

}

int lauchWork(int p_level, QString nombreFichero )
{
    // Creamos la instacia para realizar las operaciones.
    int result = 0;

    // Realizamos la operación 
    switch(p_level)
    {
    case testPackage:
        //printf("Processing test package from %s\n",nombreFichero.toAscii().data());fflush(stdout);
        //result = processTests(nombreFichero);
        break;

    case ayuda:
    default:
        showHelp();

    case test:
        printf("Performing test\n");
        printf("Calling function with file %s\n",nombreFichero.toStdString().c_str());
        //result = DoTest(nombreFichero);
        break;
    }

    // Acaba
    return result;
}

int parseOptionsAndRun(int argnum, char **args)
{
	/*
    int c;
    int p_level = nada;
    QString nombreFichero;


    while(1)
    {
       static struct option long_options[] =
         {

           {"testpkg",     required_argument,       0, 'p'},
           {"test",        optional_argument,       0, 't'},
           {"help",        no_argument,             0, 'h'},
           {0, 0, 0, 0}
         };

       int option_index = 0;
       c = getopt_long (argnum, args, "p:t:h", long_options, &option_index);

       if (c== -1)
        break;

       switch (c)
         {
         case 'p':
           // 	   p_level = 2;
           p_level = testPackage;
           nombreFichero.append(optarg);
           break;

         case 't':
           p_level = test;
//	   printf("optarg=%s\n",optarg);
           if(optarg) //! It works only if called with "--test" and not with "-t"
               nombreFichero.append(optarg);
           break;

         case 'h':
         default:
             p_level=ayuda;
             break;

        }

     }

   if (optind < argnum)
     {
       printf ("non-option ARGV-elements: ");
       while (optind < argnum)
         printf ("%s ", args[optind++]);
         putchar ('\n');
     }

   return lauchWork(p_level, nombreFichero);

   */
	return 0;

}

int main(int argc, char *argv[])
{
    // si hay par‡metros arrancamos en background sin interficie.
    if(argc > 1)
    {
        return parseOptionsAndRun(argc, argv);
    }

	RedirectIOToConsole();

	#ifdef _DEBUG

	printf("--------------------------------------------- \n");
	printf("                DEBUG MODE                    \n");
	printf("--------------------------------------------- \n");

	#endif

    // si no hay parametros arrancamos la interfaz talcual.
    QApplication a(argc, argv);
    MainWindow w;
    w.setWindowTitle("Coordinates test Toolbox");
    w.show();

    return a.exec();
}
